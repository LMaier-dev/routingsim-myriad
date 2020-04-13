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

	NetworkTopology &ntop = getTopology();

	double linkFraction = par("linkFraction").doubleValue();
	size_t numLinks = ntop.getNumberLinks() / 2; // note: getNumberLinks() returns unidirectional links, we always disable both
	unsigned int numLinksFail = linkFraction > 0.0 ? (size_t) ceil(numLinks*linkFraction + 0.5) : par("numLinksFail").intValue();
	double duration = par("duration").doubleValue();
	double period = par("period").doubleValue();
	string linkSelection = par("linkSelection").stdstringValue();
	unsigned int minDegree = par("minDegree").intValue();
	unsigned int maxDegree = par("maxDegree").intValue();
	std::map<nodeid_t, unsigned long> nodeDegrees;
	unsigned int scheduledFailures = 0;

	if (linkSelection == "none") return simTime() + period;


	// get the mapping from active nodes to active their degree
	ntop.getNodeDegrees(nodeDegrees);

	// select random node, remove all links but one if possible
	// neighboring nodes must not be isolated by choice of links (stay connected)
	if (linkSelection == "randomAllButOne") {

		unsigned long max_retries= (3 * ntop.getActiveNumberNodes())/2;
		nodeid_t source = undefined_node;
		nodeid_t destination = undefined_node;

		std::set<nodeid_t> selectedNodes;
		EV_TRACE << "linkSelection randomAllButOne(): number of bidirectional links to fail=" <<  numLinksFail << endl;
		// it can be the case that the graph is connected, but
		// we cannot remove a single link without partitioning it
		// therefore we have a maximum number of retries
		while (max_retries > 0 && scheduledFailures < numLinksFail)
		{
			max_retries--;
			
			// no more edges available
			if (ntop.getActiveNumberLinks() == 0) {
				EV_WARN << "Premature termination of Dynamics Module LinkFailure (not enough active edges!)" << endl;
				break;
			}

			// choose source node
			source = getTopology().getRandomNode(getRNG(0));
			// if source node already selected: skip
			if (selectedNodes.find(source) != selectedNodes.end())
				continue;

			std::vector<nodeid_t> neighbors;
			ntop.getNeighborNodeIDs(source, neighbors);

			// single link only nodes are not eligible or if all links are more than required
			if (neighbors.size() <= 1 or neighbors.size() > numLinksFail-scheduledFailures)
				continue;

			// check for dependent nodes (those already selected) in the neigborhood:
			// They are not eligible, otherwise we risk partioning
			bool eligible= true;
			for (unsigned int i= 0; i < neighbors.size(); i++)
				if (selectedNodes.find(neighbors[i]) != selectedNodes.end())
				{
					eligible= false;
					break;
				}
			if (not eligible)
				continue;

			unsigned int remaininglink= intuniform(0,neighbors.size()-1);
			// it may be the case that some neighboring node already chose that link to be down
			for (unsigned int i= 0; i < neighbors.size(); i++)
			{
				// safety measures: remove all but one link from this node
				// the neighboring node must have at least one more link
				// (otherwise we may get two isolated nodes connected to each other)
				if ( nodeDegrees[source] > 1 and nodeDegrees[neighbors[i]] > 2 and i != remaininglink )
				{
					destination = neighbors[i];
					scheduleLinkFailure(source, destination, period, duration);
					nodeDegrees[source]--;
					nodeDegrees[neighbors[i]]--;
					scheduledFailures++;
				}
			}
			selectedNodes.insert(source);
		} // end while
		EV_TRACE << "linkSelection randomAllButOne(): scheduled bidirectional link failures=" << scheduledFailures << endl;
		return simTime() + period;
	}

	// other modes
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
