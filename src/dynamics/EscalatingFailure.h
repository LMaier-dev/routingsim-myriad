#ifndef __ROUTINGSIM_ESCALATINGFAILURE_H_
#define __ROUTINGSIM_ESCALATINGFAILURE_H_

#include <vector>

#include <omnetpp.h>
#include "foundation/NetworkDynamics.h"

namespace routingsim {

/**
 *  Node failures in waves of growing size: wave i fails fractions[i] of the
 *  nodes at warmup + i*waveSpacing, each node staying down for
 *  downMin + U(0,downJitter). One run sweeps a whole failure-size series.
 *
 *  Every node's reboot time is recorded as a scalar (down_<node>_<wave>) so an
 *  evaluation can measure recovery from the reboot rather than from the wave.
 */
class EscalatingFailure: public NetworkDynamics {
private:
	std::vector<double> waves;
	size_t idx = 0;
protected:
	virtual simtime_t scheduleDynamics();
};

} //namespace

#endif // __ROUTINGSIM_ESCALATINGFAILURE_H_
