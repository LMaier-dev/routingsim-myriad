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

#include "NetworkManager.h"
#include "NetworkDynamics.h"

#include <boost/foreach.hpp>

namespace routingsim {

Define_Module(NetworkManager);

NetworkManager* mgr = NULL;

NetworkManager::~NetworkManager() {
	mgr = NULL;
	topology = NULL;
}

NetworkManager::NetworkManager() {

}

void NetworkManager::initialize() {
	EV << "initializing network manager" << endl;
	// set singleton variable
	mgr = this;

	// create network topology
	topology = new NetworkTopology();
	if (par("topologyFile").stdstringValue().size()!=0)
		topology->readTopology(par("topologyFile").stdstringValue());
	if (par("spathFile").stdstringValue().size()!=0)
		topology->readShortestPaths(par("spathFile").stdstringValue());
	if (par("topologyFile").stdstringValue().size()!=0) {
		topology->createNetwork(this);
	} else {
		// the network was built from a ned file
		topology->extractFromNetwork(this);
	}

	// dynamics module connected? yes-> get dynamics module!
	cGate* dynamicsGate = gate("dynamics");
	if (dynamicsGate->isPathOK()) {
		cModule* dynamicsModule =
				dynamicsGate->getPathStartGate()->getOwnerModule();
		dynamics = dynamic_cast<NetworkDynamics*>(dynamicsModule);
	}

	// no-> create using given name
	if (dynamics == NULL) {
		// create dynamics modules
		string dynamicsModel = par("dynamicsModel");
		if (dynamicsModel != string("None") ) {
			string modelName = string("routingsim.dynamics.")+dynamicsModel;
			cModuleType* type = cModuleType::get(modelName.c_str());
			cModule* mod = type->create( "dynamics", this );
			mod->finalizeParameters();

			// connect gates
			mod->gate("toManager")->connectTo( this->gate("dynamics") );

			// create internals, and schedule it
			mod->buildInside();
			mod->scheduleStart(simTime());

			dynamics = dynamic_cast<NetworkDynamics*>(mod);
		}
	}

	// initialize dynamics module
	if (dynamics != NULL) {
		EV << "Using dynamic model: " << dynamics->getModuleType()->getName() << endl;
		dynamics->manager = this;
		dynamics->topology = topology;
	} else
		EV << "No dynamic model available." << endl;
}

void NetworkManager::handleMessage(cMessage *msg) {
	if (msg->arrivedOn("dynamics")) {
		EV << "received dynamics message" << endl;

		// if it's a selfmessage, then we have DynamicsInfo attached
		DynamicsInfo *di =
				static_cast<DynamicsInfo *> (msg->getContextPointer());
		assert(di!=NULL);

		switch (di->type) {
		case DynamicsInfo::nodeFailure:
			topology->disableNode(di->source);
			resetNode(di->source);
			break;
		case DynamicsInfo::nodeRecovery:
			topology->enableNode(di->source);
			break;
		case DynamicsInfo::linkFailure:
			break;
		case DynamicsInfo::linkRecovery:
			break;
		case DynamicsInfo::undefined:
			EV << "undefined event, did you forget to set DynamicsInfo::type?"
			<< endl;
			break;
		default:
			EV << "unknown dynamic event" << endl;
		}

		delete di;
		delete msg;
	}
}

distance_t NetworkManager::getDistance(cModule *from, cModule* to) {
	return topology->getDistance(from, to);
}

int NetworkManager::getNetworkSize() {
	if (topology == NULL) {
		throw cRuntimeError("topology has not been created yet!");
	}
	topology->getNetworkSize();
}

void NetworkManager::finish() {
	delete topology;
}

void NetworkManager::resetNode(nodeid_t n) {
	cModule *node = topology->NodeIDToModule(n);
	int size = node->gate("toProtocols$o", 0)->size();

	for (int i = 0; i < size; i++) {
		cGate *tnm = node->gate("toProtocols$o", i);
		check_and_cast<Protocol *> (tnm->getNextGate()->getOwner())->reset();
	}
}

NetworkManager* getNetworkManager() {
	if (mgr == NULL) {
		cModuleType* type = cModuleType::get("routingsim.foundation.NetworkManager");
		cModule* mod = simulation.getContextModule();
		while(mod->getParentModule()!=NULL) mod = mod->getParentModule();
		mgr = dynamic_cast<NetworkManager*>(type->create( "NetworkManager",mod ));
		if (mgr == NULL)
			throw cRuntimeError("Initialization of NetworkManager failed");
	}
	return mgr;
}

} //namespace
