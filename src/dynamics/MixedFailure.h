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

#ifndef __ROUTINGSIM_CATASTROPHICFAILURE_H_
#define __ROUTINGSIM_CATASTROPHICFAILURE_H_

#include <omnetpp.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include "foundation/NetworkDynamics.h"
#include "NetworkManager.h"

namespace routingsim {

/**
 * This class implements an incremental failure model to, e.g.,
 * track the performance of a routing protocol in a consistently
 * degrading environment
 *
 *  @author Christoph Werle <werle@kit.edu>
 */
class MixedFailure: public NetworkDynamics {
private:
	bool first;
	unsigned int iteration;
	// parameters from ned file
	double warmup;

	bool nodeFailure;
	double nodeFraction;
	double nodeFailPeriod;
	double nodeFailDuration;
	double nodeFailDurationJitter;

	bool linkFailure;
	double linkFraction;
	double linkFailPeriod;
	double linkFailDuration;
	double linkFailDurationJitter;

	// rescheduling variables
	double nodeFailureDue;
	double linkFailureDue;
	std::ofstream linkFailOut;

protected:
	virtual void initialize();
	virtual simtime_t scheduleDynamics();

public:
	virtual ~MixedFailure();
};

} //namespace

#endif
