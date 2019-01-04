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

using namespace boost;

Define_Module(CatastrophicFailure);

void CatastrophicFailure::initialize() {
	first = true;
}


/**
 * Disable the given percentage of nodes, disregarding the connectivity of the
 * graph and re-enable them again after catastrophicDuration.
 *
 * Currently also no check for multiple selection of a node is performed,
 * i.e., the same node could be selected over and over again leading to a
 * smaller amount of nodes that fail.
 *
 * TODO: Add check for duplicates
 */
simtime_t CatastrophicFailure::scheduleDynamics() {
	// get parameters
	double fraction = par("fraction").doubleValue();
	//assert(fraction>0);
	double duration = par("duration").doubleValue();
	assert(duration>0);
	double period = par("period").doubleValue();
	double warmup = par("warmup").doubleValue();
	size_t minDegree = par("minDegree");
	size_t maxDegree = par("maxDegree");

	// determine nodes to fail
	size_t numNodesFail = ((double) getTopology().getNumberNodes() * fraction);
	EV << "Scheduling " << numNodesFail
	   << " nodes for catastrophic failure"	<< endl;

	unordered_set<nodeid_t> alreadyFailed;
	for (size_t i = 0; i < numNodesFail; i++) {

		// find a node for failure
		nodeid_t node;
		bool cond;
		size_t degree;
		do {
			node = getTopology().getRandomNode(intuniform(0, 65536));
			degree = getTopology().getNodeDegree(node);
			cond = true;
			cond &= (alreadyFailed.count(node) != 0);
			cond &= (maxDegree == 0 || degree <= maxDegree);
			cond &= (degree >= minDegree);
		} while (cond);
		alreadyFailed.insert(node);

		// schedule node failure
		scheduleNodeFailure(node,first ? warmup : period,duration);
	}

	/// the catastrophic failure is repeated each occurrence seconds
	first=false;
	return simTime()+period;
}

// ==========================================================================
// OLD STUFF / NOTES
// ==========================================================================
#if 0
// what do I need?
/*
 * Number of Nodes
 * Simulation Time
 * Failure Types:
 * 	Single Node Failure: bool
 * 	Single Link Failure: bool
 * 		- Flapping Links
 * 	Cascading Node Failure: bool
 * 	Cascading Link Failure: bool
 * 	Cascading Failure Size:
 * 	Cascading Failure Propagation:
 * 	Occurrence time: Periodic, probabilistic
 * 	Failure Duration: Static, probabilistic
 * 	Recovery from Dynamic Failure: synchronized or not?
 *
 * 	Selection Criteria??? Random, Rank, Betweenness, ... update for new topology?
 *
 */

// generate disable/enable data
// location (Node or Node+Link)
// occurrence (simtime)+ duration
// schedule disable message and enable message

// how to deal intersection of events???

/* simplest case node failure assuming node ids from 0...numberNodes-1
 * 	- known simulation time
 * 	- single failure interval (SFI) ( 10s )
 * 	for (int i=0; (i+1)*SFI < totalSimtime; i++) {
 * 	- randomize node to fail within that interval
 * 		intrand(numberNodes)
 * 	- randomize failure occurrence within that interval
 * 		occurence = simTime() + i*SFI + dblrand()*SFI
 * 	- randomize duration within interval-occurrence
 * 		duration = simTime() + (i+1)*SFI - occurrence
 * 	- scheduleAt(occurence,
 */

for (int i=0; (i+1)*SFI < totalSimtime; i++) {
	simtime_t occurrence, duration;

	int failedNode = intrand(nodes);
	occurrence = simTime() + i*SFI + dblrand()*SFI;
	duration = dblrand() * ( simTime() + (i+1)*SFI - occurrence );

	// prepare and schedule self message for failure
	DynamicsInfo *diFail = new DynamicsInfo();
	diFail->update_type = DynamicsInfo::nodeFailure;
	diFail->source = failedNode;
	diFail->occurrence = occurrence;
	diFail->duration = duration;

	dynamicEventMessage = new cMessage();
	dynamicEventMessage->setContextPointer(diFail);

	sendDelayed(dynamicEventMessage, diFail->occurrence - simTime(), "NetworkManager");
	EV << "scheduling node failure at " << diFail->occurrence << endl;
	// prepare and schedule self message for recovery
	DynamicsInfo *diRecovery = new DynamicsInfo();
	diRecovery->update_type = DynamicsInfo::nodeRecovery;
	diRecovery->source = failedNode;
	diRecovery->occurrence = occurrence+duration;
	diRecovery->duration = duration;

	dynamicEventMessage = new cMessage();
	dynamicEventMessage->setContextPointer(diRecovery);

	sendDelayed(dynamicEventMessage, diRecovery->occurrence - simTime(), "NetworkManager");
	EV << "scheduling node recovery at " << diRecovery->occurrence << endl;
}
#endif

} //namespace
