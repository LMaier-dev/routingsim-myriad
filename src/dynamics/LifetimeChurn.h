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

#ifndef __ROUTINGSIM_LIFETIMECHURN_H_
#define __ROUTINGSIM_LIFETIMECHURN_H_

#include <omnetpp.h>
#include "NetworkDynamics.h"

namespace routingsim {

/**
 * This module implements a distributed churn dynamics model.
 * It supports several distribution functions, such as weibull, trucnormal
 *
 * @author Sebastian Mies <mies@kit.edu>
 */
class LifetimeChurn: public NetworkDynamics {
protected:
	virtual void initialize();
	virtual simtime_t scheduleDynamics();

private:
	string distribution;
	double distParameter;
	double meanLifetime;

	class node_state_t {
	public:
		nodeid_t id;
		simtime_t upagain;
	};
	typedef vector<node_state_t> state_t;
	state_t state;

	double distributionFunction();
};

} //namespace

#endif
