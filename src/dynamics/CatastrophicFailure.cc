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

#include "CatastrophicFailure.h"

#include <boost/unordered_set.hpp>

namespace routingsim {

Define_Module(CatastrophicFailure);

void CatastrophicFailure::initialize() {
	first = true;
}

/**
 * Disable the given percentage of avtive nodes, disregarding the connectivity of the
 * graph and re-enable them again after catastrophicDuration.
 */
simtime_t CatastrophicFailure::scheduleDynamics() {
	// get parameters
	double fraction = par("fraction").doubleValue();
	double duration = par("duration").doubleValue();
	double period = par("period").doubleValue();
	double warmup = par("warmup").doubleValue();
	size_t minDegree = par("minDegree");
	size_t maxDegree = par("maxDegree");

	assert(fraction>=0);
	assert(duration>0);

	// maxDegree 0 --> consider all nodes by setting maxDegree to max!
	if (maxDegree == 0) {
		maxDegree = std::numeric_limits<int>::max();
	}

	// acquire list of nodes and select potential targets
	size_t numNodesFail = ((double) getTopology().getActiveNumberNodes() * fraction);
	EV << "Scheduling " << numNodesFail << " nodes for catastrophic failure" << endl;

	map<nodeid_t, unsigned long> nodeDegreeMap;
	size_t nodeDegreeMapSize;
	map<nodeid_t, unsigned long>::iterator ndit;

	getTopology().getNodeDegrees(nodeDegreeMap);
	for (ndit=nodeDegreeMap.begin(); ndit!=nodeDegreeMap.end();) {
		if (ndit->second <= minDegree || ndit->second >= maxDegree) {
			nodeDegreeMap.erase(ndit++);
		} else {
			++ndit;
		}
	}
	nodeDegreeMapSize = nodeDegreeMap.size();

	// determine actual nodes to fail
	if (nodeDegreeMapSize < numNodesFail) {
		EV << "Failing only " << nodeDegreeMapSize << " nodes as these are the only "
		"ones within the degree range ["<<minDegree<<","<<maxDegree<<"]!" << endl;
	} else {
		EV << "Scheduling " << numNodesFail << " nodes for catastrophic failure" << endl;
	}

	for (size_t i = 0; i < min(numNodesFail, nodeDegreeMapSize); i++) {
		size_t rnd = getRNG(0)->intRand(nodeDegreeMap.size());
		std::advance(ndit=nodeDegreeMap.begin(), rnd);
		nodeid_t rndnode = ndit->first;
		nodeDegreeMap.erase(ndit);

		// schedule node failure
		scheduleNodeFailure(rndnode,first ? warmup : period,duration);
	}

	/// the catastrophic failure is repeated each occurrence seconds
	first = false;
	return simTime() + period;
}

} //namespace
