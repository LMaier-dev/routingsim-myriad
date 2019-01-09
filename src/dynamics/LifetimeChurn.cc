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

#include "LifetimeChurn.h"

#include<boost/foreach.hpp>

namespace routingsim {

using namespace boost;

Define_Module(LifetimeChurn);

void LifetimeChurn::initialize() {
	// get parameters
	distribution 	= par("distribution").stdstringValue();
	distParameter 	= par("distParameter");
	meanLifetime	= par("meanLifetime");

	// get min/max-degree
    int minDegree 	= par("minDegree");
    int maxDegree	= par("maxDegree");

    // extract relevant nodes
	vector<nodeid_t> ids;
	getTopology().getNodeIDs( ids );

	foreach(nodeid_t id, ids) {
		int deg = getTopology().getNodeDegree(id);
		if (deg>=minDegree && (deg<=maxDegree || maxDegree==0)) {
			node_state_t nstate;
			nstate.id = id;
			nstate.upagain = simTime();
			state.push_back(nstate);
		}
	}
}

simtime_t LifetimeChurn::scheduleDynamics() {
	simtime_t st = simTime();
	simtime_t nextSchedule = st;
	foreach(node_state_t& ns, state) {
		if (ns.upagain+10 >=  st) {
			double df = distributionFunction();
			scheduleNodeFailure(ns.id, ns.upagain+df, df );
			ns.upagain += (df * 2);
			if (nextSchedule == st || nextSchedule > ns.upagain-5)
				nextSchedule = (ns.upagain-5) <= simTime() ? simTime()+5 : ns.upagain-5;
		}
	}
	return nextSchedule;
}

double LifetimeChurn::distributionFunction() {

	// weibull
	if ( distribution == "Weibull") {
	    double p = meanLifetime / tgamma(1 + (1 / distParameter));
	    return weibull(p, distParameter);
	}

	// pareto shifted
	if ( distribution == "ParetoShifted") {
		double p = meanLifetime * (distParameter - 1) / distParameter;
	    return pareto_shifted(distParameter, p, 0);
	}

	// trunc normal
	if ( distribution == "TruncNormal") {
		double p = meanLifetime;
	    return truncnormal(p, p/3.0);
	}

	throw cRuntimeError("LifetimeChurn::distribution function: "
			  "Invalid value for parameter distribution!");

	return 0.0;
}


} //namespace
