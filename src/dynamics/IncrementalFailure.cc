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

#include "IncrementalFailure.h"

#include <boost/unordered_set.hpp>

namespace routingsim {

Define_Module(IncrementalFailure);

void IncrementalFailure::initialize() {
	first = true;
	iteration = 0;
}

/**
 * Incrementally disable the given percentage of nodes and links starting
 * at time warmup and from then repeating each period seconds
 */
simtime_t IncrementalFailure::scheduleDynamics() {

	if (iteration == 0) {
		// get parameters
		NodeFailure = par("NodeFailure").boolValue();
		LinkFailure = par("LinkFailure").boolValue();
		NodeFraction = par("NodeFraction").doubleValue();
		LinkFraction = par("LinkFraction").doubleValue();
		warmup = par("warmup").doubleValue();
		period = par("period").doubleValue();
	}
	NetworkTopology &topology = getNetworkManager()->getTopology();

	size_t numNodes = topology.getNumberNodes();
	size_t numLinks = topology.getNumberLinks();
	size_t activeNumLinks = topology.getActiveNumberLinks();
	size_t nodesToFailPerPeriod = NodeFailure ? (size_t) numNodes*NodeFraction : 0;
	size_t linksToFailPerPeriod = LinkFailure ? (size_t) numLinks*LinkFraction : 0;

	vector<nodeid_t> activeNodes;
	topology.getNodeIDs( activeNodes );

	// determine actual nodes to fail
	if ( NodeFailure ) {
		vector<nodeid_t> failNodes;

		if (activeNodes.size() < nodesToFailPerPeriod) {
		    // no more nodes available
			EV_WARN << "Premature termination of Dynamics Module LinkFailure (not enough active nodes!); failing remaining nodes and disabling dynamics" << endl;
			failNodes = activeNodes;
			NodeFailure=false;
		} else {
		    // select nodes
			for (size_t i=0; i<nodesToFailPerPeriod; i++) {
				unsigned int idx = intuniform(0, activeNodes.size()-1);
				failNodes.push_back( activeNodes[idx] );
				activeNodes.erase(activeNodes.begin()+idx);
			}
		}

		EV << "SIMTIME: " << simTime() << "\t";
		EV << "Failing " << failNodes.size() << " nodes at IncrementalFailure Iteration " << iteration << endl;

		// deactivate nodes
		for (size_t i=0; i<failNodes.size(); i++) {
			EV << "Failing node " << failNodes[i] << endl;
			scheduleNodeFailure(failNodes[i], first ? warmup : period, SIMTIME_MAX);
		}
	}

	// determine actual links to fail; in case of both---link and node failures---
	// link failures happen after node failures
	if ( LinkFailure ) {
		vector< pair<nodeid_t, nodeid_t> > failEdges;
		// update number of active links, might have changed
		activeNumLinks = topology.getActiveNumberLinks();

		// no more edges available
		if (activeNumLinks < linksToFailPerPeriod) {
			EV_WARN << "Premature termination of Dynamics Module LinkFailure (not enough active edges!); failing remaining edges and disabling dynamics" << endl;
			linksToFailPerPeriod = activeNumLinks;
			LinkFailure=false;
		}

		while ( failEdges.size() < linksToFailPerPeriod ) {
			pair<nodeid_t, nodeid_t> failEdge = topology.getRandomEdge( intuniform(0,activeNumLinks-1) );
			if ( find(failEdges.begin(), failEdges.end(), failEdge) != failEdges.end() ) {
				continue;
			} else {
				failEdges.push_back(failEdge);
			}
		}
		EV << "SIMTIME: " << simTime() << "\t";
		EV << "Failing " << failEdges.size() << " links at IncrementalFailure Iteration " << iteration << endl;

		// deactivate edges
		for (size_t i=0; i<failEdges.size(); i++) {
			EV << "Failing edge (" << failEdges[i].first << ", " << failEdges[i].second << ")" << endl;
			scheduleLinkFailure(failEdges[i].first, failEdges[i].second, first ? warmup : period, SIMTIME_MAX);
		}
	}

	++iteration;

	// handle warmup phase
	if (first) {
		first=false;
		return simTime()+warmup;
	}

	/// the catastrophic failure is repeated each occurrence seconds
	return simTime()+period;
}

} //namespace
