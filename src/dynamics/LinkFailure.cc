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

#include "LinkFailure.h"
#include "NetworkDynamics.h"
#include <utility>
#include <list>
#include <string>
#include <iostream>

namespace routingsim {

Define_Module(LinkFailure);


simtime_t LinkFailure::scheduleDynamics() {

	unsigned int numLinksFail = par("numLinksFail").intValue();
	double duration = par("duration").doubleValue();
	double period = par("period").doubleValue();
	string linkSelection = par("linkSelection").stdstringValue();
	unsigned int minDegree = par("minDegree").intValue();
	unsigned int maxDegree = par("maxDegree").intValue();
	std::map<nodeid_t, unsigned long> nodeDegrees;
	unsigned int scheduledFailures = 0;

	if (linkSelection == "none") return simTime() + period;

	NetworkTopology &ntop = getTopology();

	// get the mapping from active nodes to active their degree
	ntop.getNodeDegrees(nodeDegrees);

	while (scheduledFailures < numLinksFail) {
		nodeid_t source = undefined_node;
		nodeid_t destination = undefined_node;

		// no more edges available
		if (ntop.getActiveNumberLinks() == 0) {
			EV_WARN <<"Premature termination of Dynamics Module LinkFailure (not enough active edges!)" << endl;
			break;
		}

		if (linkSelection == "degreeRange") {
			// select numLinksFail links connected to a node with degree in the range [minDegree, maxDegree]
			std::map<nodeid_t, unsigned long> nodesInDegreeRange;
			std::map<nodeid_t, unsigned long>::iterator it;
			std::vector<nodeid_t> neighbors;

			// select nodes within degree range
			for (it = nodeDegrees.begin(); it != nodeDegrees.end(); it++) {
				if (it->second <= maxDegree && it->second >= minDegree) {
					nodesInDegreeRange.insert(*it);
				}
			}
			// select link by source/destination
			source = nodesInDegreeRange[intuniform(0, nodesInDegreeRange.size() - 1)];
			ntop.getNeighborNodeIDs(source, neighbors);
			destination = neighbors[intuniform(0, neighbors.size() - 1)];
		} else if (linkSelection == "random") {
			std::vector<nodeid_t> neighbors;

			source = getTopology().getRandomNode(getRNG(0));
			ntop.getNeighborNodeIDs(source, neighbors);
			if (neighbors.size()==0)
                        	continue;

			destination = neighbors[intuniform(0, neighbors.size() - 1)];
		} else {
			throw cRuntimeError("Invalid Link Failure model!");
		}
		scheduleLinkFailure(source, destination, period, duration);
		scheduledFailures++;
	}

	/// the failure is repeated each occurrence seconds
	return simTime() + period;
}

} /* namespace routingsim */
