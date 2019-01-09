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
		scheduleAt(nextSchedule, timer);
	}
}

void NetworkDynamics::initialize() {

}

void NetworkDynamics::finish() {
}

NetworkDynamics::~NetworkDynamics() {
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
		stringstream fls;
		fls << "Fail Node " << di->source;
		string failLabel(fls.str());
		dim = new cMessage(failLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");

	di = new DynamicsInfo();
	di->type = DynamicsInfo::nodeRecovery;
	di->source = node;
	di->occurrence = (duration==SIMTIME_MAX ? SIMTIME_MAX : occurrence + duration);
	di->duration = duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		stringstream rls;
		rls << "Recovery Node " << di->source;
		string recoverLabel(rls.str());
		dim = new cMessage(recoverLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);

	// if nodes fail forever, do not schedule the recovery msg but free msgs
	if (duration != SIMTIME_MAX) {
		sendDelayed(dim, di->occurrence, "toManager");
	} else {
		delete di; delete dim;
	}
}

void NetworkDynamics::scheduleLinkFailure(nodeid_t from, nodeid_t to, simtime_t occurrence, simtime_t duration) {
	DynamicsInfo* di;
	cMessage* dim;

	di = new DynamicsInfo();
	di->type = DynamicsInfo::linkFailure;
	di->source = from;
	di->destination = to;
	di->occurrence = occurrence;
	di->duration = duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		stringstream fls;
		fls << "Fail Link [" << from << "," << to << "]";
		string failLabel(fls.str());
		dim = new cMessage(failLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");

	di = new DynamicsInfo();
	di->type = DynamicsInfo::linkRecovery;
	di->source = from;
	di->destination = to;
	di->occurrence = (duration==SIMTIME_MAX ? SIMTIME_MAX : occurrence + duration);
	di->duration = duration;
	if (cSimulation::getActiveEnvir()->isGUI()) {
		stringstream rls;
		rls << "Recovery Link [" << from << "," << to << "]";
		string recoverLabel(rls.str());
		dim = new cMessage(recoverLabel.c_str());
	} else {
		dim = new cMessage();
	}
	dim->setContextPointer(di);

	// if nodes fail forever, do not schedule the recovery msg but free msgs
	if (duration != SIMTIME_MAX) {
		sendDelayed(dim, di->occurrence, "toManager");
	} else {
		delete di; delete dim;
	}
}

/*
 * handle scheduled dynamic event messages
 */
void NetworkDynamics::handleMessage(cMessage *msg) {
	ASSERT(msg==timer);
	simtime_t nextSchedule = this->scheduleDynamics();
	scheduleAt(nextSchedule,timer);
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
