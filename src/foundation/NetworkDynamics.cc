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
		scheduleAt(nextSchedule,timer);
	}
}

void NetworkDynamics::initialize() {

}


void NetworkDynamics::finish() {
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
	dim = new cMessage("nodeFailure");
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");

	di = new DynamicsInfo();
	di->type = DynamicsInfo::nodeRecovery;
	di->source = node;
	di->occurrence = occurrence + duration;
	di->duration = duration;
	dim = new cMessage("nodeRecovery");
	dim->setContextPointer(di);
	sendDelayed(dim, di->occurrence, "toManager");
}

/*
 * handle scheduled dynamic event messages
 */
void NetworkDynamics::handleMessage(cMessage *msg) {
	ASSERT(msg==timer);
	simtime_t nextSchedule = this->scheduleDynamics();
	scheduleAt(nextSchedule,timer);
}

} //namespace
