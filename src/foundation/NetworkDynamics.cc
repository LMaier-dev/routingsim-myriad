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

#include "NetworkDynamics.h"
#include "NetworkTopology.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace routingsim {

//Define_Module(NetworkDynamics);

void NetworkDynamics::initialize(int stage) {
	if (stage == 0) {
		timer = new cMessage("networkDynamicsTimer");
		initialize();
	}

	if (stage == 1) {
		initialize();
		simtime_t nextSchedule = this->scheduleDynamics();
		if (nextSchedule < SIMTIME_MAX) {
			scheduleAt(nextSchedule, timer);
		}
		else { // no subsequent event
			delete timer;
			timer= nullptr;
		}
	}
}

void NetworkDynamics::initialize() {
}

void NetworkDynamics::finish() {
}

NetworkDynamics::~NetworkDynamics() {
  if (timer != nullptr)
    cancelAndDelete(timer);
}

int NetworkDynamics::numInitStages() const {
	return 2;
}

void NetworkDynamics::scheduleNodeFailure(nodeid_t node, simtime_t occurrence,
		simtime_t duration) {
	DynamicsInfo* di;
	cMessage* dim;

	di = new DynamicsInfo();
	di->type = DynamicsInfo::nodeFailure;
	di->source = node;
	di->occurrence = occurrence;
	di->duration = duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		std::stringstream fls;
		fls << "Fail Node " << di->source;
		string failLabel(fls.str());
		dim = new cMessage(failLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");
	// if node fails forever, do not schedule recovery msg
	if (duration == SIMTIME_MAX) return;

	di = new DynamicsInfo(*di);
	di->type = DynamicsInfo::nodeRecovery;
	di->occurrence += duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		std::stringstream rls;
		rls << "Recovery Node " << di->source;
		string recoverLabel(rls.str());
		dim = new cMessage(recoverLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");
}


void
NetworkDynamics::scheduleLinkFailure(nodeid_t from, nodeid_t to, simtime_t occurrence, simtime_t duration, bool bidir) {
	DynamicsInfo* di;
	cMessage* dim;

	di = new DynamicsInfo();
	di->type = DynamicsInfo::linkFailure;
	di->source = from;
	di->destination = to;
	di->occurrence = occurrence;
	di->duration = duration;
	di->bidirectionalLinkFailure = bidir;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		std::stringstream fls;
		fls << "Fail Link [" << from << "," << to << "]";
		string failLabel(fls.str());
		dim = new cMessage(failLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");
	// if link fails forever, do not schedule recovery msg
	if (duration == SIMTIME_MAX) return;

	di = new DynamicsInfo(*di); // copy failure info
	di->type = DynamicsInfo::linkRecovery;
	di->occurrence += duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		std::stringstream rls;
		rls << "Recovery Link [" << from << "," << to << "]";
		string recoverLabel(rls.str());
		dim = new cMessage(recoverLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");
}

/*
 * handle scheduled dynamic event messages
 */
void NetworkDynamics::handleMessage(cMessage *msg) {
	ASSERT(msg==timer);
	simtime_t nextSchedule = this->scheduleDynamics();
	if (nextSchedule < SIMTIME_MAX) {
		scheduleAt(nextSchedule,timer);
	}
	else { // no subsequent event
		delete msg;
		timer= nullptr;
	}
}

/*
 *
 *
 * vector_set<nodeid_t>neighbours = di->source->getNeighbours();
 int random = rand() % neighbours;
 di->destination = neighbours[random];
 deletedNoteLinks.insert(di->destination);


 di->destination = deletedNoteLinks[0];
 deletedNoteLinks.erase(deletedNoteLinks[0]);

 */
} //namespace
