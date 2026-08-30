#ifndef __ROUTINGSIM_NODECHURN_H_
#define __ROUTINGSIM_NODECHURN_H_

#include <omnetpp.h>
#include "foundation/NetworkDynamics.h"

namespace routingsim {

/**
 *  Steady low-rate node churn: numNodes random nodes fail every period, each
 *  down for downMin + U(0,downJitter), forever. A background failure rate the
 *  network absorbs, as opposed to EscalatingFailure's growing waves.
 */
class NodeChurn: public NetworkDynamics {
private:
	bool first = true;
protected:
	virtual simtime_t scheduleDynamics();
};

} //namespace

#endif // __ROUTINGSIM_NODECHURN_H_
