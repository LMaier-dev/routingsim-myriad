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

#include "MixedFailure.h"

#include <boost/unordered_set.hpp>

namespace routingsim {

using namespace boost;

Define_Module(MixedFailure);

void MixedFailure::initialize() {
	first = true;
	iteration = 0;
	nodeFailureDue = -1;
	linkFailureDue = -1;
	string prefix = "results/"+string(getEnvir()->getConfigEx()->getActiveConfigName())+"-"+boost::lexical_cast<string>(getEnvir()->getConfigEx()->getActiveRunNumber());
	linkFailOut.open( (prefix+"-failedLinks").c_str() );
}

MixedFailure::~MixedFailure() {
	linkFailOut.close();
}

/**
 * Incrementally disable the given percentage of nodes and links starting
 * at time warmup and from then repeating each period seconds
 */
simtime_t MixedFailure::scheduleDynamics() {

	if (iteration == 0) {
		// get parameters
		warmup = par("warmup").doubleValue();

		nodeFailure = par("nodeFailure").boolValue();
		nodeFraction = par("nodeFraction").doubleValue();
		nodeFailPeriod = par("nodeFailPeriod").doubleValue();
		nodeFailDuration = par("nodeFailDuration").doubleValue();
		nodeFailDurationJitter = par("nodeFailDurationJitter").doubleValue();

		linkFailure = par("linkFailure").boolValue();
		linkFraction = par("linkFraction").doubleValue();
		linkFailPeriod = par("linkFailPeriod").doubleValue();
		linkFailDuration = par("linkFailDuration").doubleValue();
		linkFailDurationJitter = par("linkFailDurationJitter").doubleValue();

	}
	NetworkTopology &topology = getNetworkManager()->getTopology();

	size_t numNodes = topology.getNumberNodes();
	size_t numLinks = topology.getNumberLinks();
	size_t activeNumLinks = topology.getActiveNumberLinks();
	size_t nodesToFailPerPeriod = nodeFailure ? (size_t) numNodes*nodeFraction : 0;
	size_t linksToFailPerPeriod = linkFailure ? (size_t) numLinks*linkFraction : 0;

	vector<nodeid_t> activeNodes;
	topology.getNodeIDs( activeNodes );

	// determine actual nodes to fail
	if ( nodeFailure && simTime()==nodeFailureDue ) {
		vector<nodeid_t> failNodes;
		failNodes.reserve(nodesToFailPerPeriod);

		// no more nodes available
		if (activeNodes.size() < nodesToFailPerPeriod) {
			EV_WARN << "Premature termination of Dynamics Module (not enough active nodes!); failing remaining nodes and disabling dynamics" << endl;
			failNodes = activeNodes;
			nodeFailure=false;
		} else {
			for (size_t i=0; i<nodesToFailPerPeriod; i++) {
				unsigned int idx = intuniform(0, activeNodes.size()-1);
				failNodes.push_back( activeNodes[idx] );
				activeNodes.erase(activeNodes.begin()+idx);
			}
		}

		EV << "SIMTIME: " <<simTime()<< "\t";
		EV << "Failing " << failNodes.size() << " nodes at MixedFailure Iteration " << iteration << endl;

		// deactivate nodes
		for (size_t i=0; i<failNodes.size(); i++) {
			// +/- jitter
			double jitter = nodeFailDurationJitter==0 ? 0 : dblrand()*2*nodeFailDurationJitter - nodeFailDurationJitter;
			EV << "Scheduling node for failure " << failNodes[i] << endl;
			scheduleNodeFailure(failNodes[i], first ? warmup : nodeFailPeriod, nodeFailDuration+jitter);
		}
		nodeFailureDue = simTime().dbl()+nodeFailPeriod;
	}

	// determine actual links to fail; in case of both---link and node failures---
	// link failures happen after node failures
	if ( linkFailure && simTime()==linkFailureDue ) {
		vector< pair<nodeid_t, nodeid_t> > failEdges;
		failEdges.reserve(linksToFailPerPeriod);
		// update active number of links; might have changed due to node failures by now
		activeNumLinks = topology.getActiveNumberLinks();

		// no more edges available
		if (activeNumLinks < linksToFailPerPeriod) {
			EV_WARN << "Premature termination of Dynamics Module (not enough active edges!); failing remaining edges and disabling dynamics" << endl;
			linksToFailPerPeriod = activeNumLinks;
			linkFailure=false;
		}

		while ( failEdges.size() < linksToFailPerPeriod ) {
			pair<nodeid_t, nodeid_t> failEdge = topology.getRandomEdge( intuniform(0,activeNumLinks-1) );
			if ( find(failEdges.begin(), failEdges.end(), failEdge) != failEdges.end() ) {
				continue;
			} else {
				linkFailOut << failEdge.first << "\t" << failEdge.second << endl;
				failEdges.push_back(failEdge);
			}
		}
		EV << "SIMTIME: " <<simTime()<< "\t";
		EV << "Scheduling " << failEdges.size() << " links for failure at MixedFailure Iteration " << iteration << endl;

		// deactivate edges
		for (size_t i=0; i<failEdges.size(); i++) {
			// +/- jitter
			double jitter = linkFailDurationJitter==0 ? 0 : dblrand()*2*linkFailDurationJitter - linkFailDurationJitter;
			EV << "Scheduling edge (" << failEdges[i].first << ", " << failEdges[i].second << ") for failure" << endl;
			scheduleLinkFailure(failEdges[i].first, failEdges[i].second, first ? warmup : linkFailPeriod, linkFailDuration+jitter);
		}
		linkFailureDue = simTime().dbl() + linkFailPeriod;
	}

	++iteration;

	// handle warmup phase
	if (first) {
		first=false;
		nodeFailureDue = simTime().dbl()+warmup;
		linkFailureDue = simTime().dbl()+warmup;
		return simTime()+warmup;
	}

	/// the catastrophic failure is repeated each occurrence seconds
	return min( nodeFailure?nodeFailureDue:SIMTIME_MAX, linkFailure?linkFailureDue:SIMTIME_MAX);
}

} //namespace
