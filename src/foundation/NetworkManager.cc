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
	delete topology;
	mgr = NULL;
	topology = NULL;

	// clean up statistics
	delete activeEdgesCount;
	delete activeNodesCount;
	delete partitionCount;
	activeEdgesCount = activeNodesCount = partitionCount = NULL;
}

NetworkManager::NetworkManager() {
	// TODO: assert that there has not been another NetworkManager instantiated already
	evaluateStatistics = true;
	activeEdgesCount = activeNodesCount = partitionCount = NULL;
}

void NetworkManager::initialize() {
	EV << "initializing network manager" << endl;
	// set singleton variable
	mgr = this;

	// create network topology
	topology = new NetworkTopology();
	if (par("topologyFile").stdstringValue().size()!=0)
		topology->readTopology(
				par("topologyFile").stdstringValue(),
				par("topologyFileType").stdstringValue(),
				par("useOnlyLargestConnectedComponent").boolValue(),
				par("makeLinksBidirectional").boolValue(),
				par("allowNodeIdRemapping").boolValue(),
				par("compressNodeIDs").boolValue()
				);
	if (par("spathFile").stdstringValue().size()!=0)
		topology->readShortestPaths(par("spathFile").stdstringValue());
	if (par("topologyFile").stdstringValue().size()!=0) {
		topology->createNetwork(this);
	} else {
		// the network was built from a ned file
		topology->extractFromNetwork(this);
	}

	topology->setShortestPathCalculationMode(par("shortestPathCalculationMode").stdstringValue());
	cout << "shortestPathCalculationMode: " << par("shortestPathCalculationMode").stdstringValue() << endl;

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
		ev << "Using dynamic model: " << dynamics->getModuleType()->getName() << endl;
		dynamics->manager = this;
		dynamics->topology = topology;
	} else
		ev << "No dynamic model available." << endl;


	// statistics
	activeNodesCount = new cOutVector("activeNodesCount");
	activeEdgesCount = new cOutVector("activeEdgesCount");
	partitionCount = new cOutVector("partitionCount");
	addStatisticSample(this);
}

void NetworkManager::handleMessage(cMessage *msg) {
	if (msg->arrivedOn("dynamics")) {
		EV << "received dynamics message" << endl;

		// if it's a selfmessage, then we have DynamicsInfo attached
		DynamicsInfo *di =
				static_cast<DynamicsInfo *> (msg->getContextPointer());
		assert(di!=NULL);

		evaluateStatistics = true;
		switch (di->type) {
		case DynamicsInfo::nodeFailure:
			topology->disableNode(di->source);
			resetNode(di->source);
			break;
		case DynamicsInfo::nodeRecovery:
			topology->enableNode(di->source);
			break;
		case DynamicsInfo::linkFailure:
			// depending on bidirectional flag deactivate one or both directions
			topology->disableLink(di->source,di->destination);
			if (di->bidirectionalLinkFailure) {
				topology->disableLink(di->destination,di->source);
			}
			break;
		case DynamicsInfo::linkRecovery:
			// depending on bidirectional flag deactivate one or both directions
			topology->enableLink(di->source,di->destination);
			if (di->bidirectionalLinkFailure) {
				topology->enableLink(di->destination,di->source);
			}
			break;
		case DynamicsInfo::undefined:
			opp_error ("undefined event, did you forget to set DynamicsInfo::type?");
			break;
		default:
			opp_error ("undefined event, did you forget to set DynamicsInfo::type?");
			break;
		}

		delete di;
		delete msg;
	}
}

float NetworkManager::getDistance(nodeid_t from, nodeid_t to) {
	return topology->getDistance(from, to);
}

unsigned int NetworkManager::getNetworkSize() {
	if (topology == NULL) {
		opp_error("topology has not been created yet!");
	}
	return topology->getNetworkSize();
}

void NetworkManager::finish() {
}

void NetworkManager::resetNode(nodeid_t n) {
	getTopology().resetNode(n);
}

void NetworkManager::sampleStatistics( Statistics& stats ) {
	string topologyStatistics = par("topologyStatistics").stdstringValue();
	static unsigned int ann, anl, pc;

	if (topologyStatistics == string("None")) return;

	if (evaluateStatistics) {
		if (topologyStatistics == string("Light") || topologyStatistics== string("Heavy")) {
			ann = getNetworkManager()->getTopology().getActiveNumberNodes();
			anl = getNetworkManager()->getTopology().getActiveNumberLinks();
		}
		if (topologyStatistics== string("Heavy")) {
			pc = getNetworkManager()->getTopology().getNumPartitions();
		}
		evaluateStatistics = false;
	}


	if (topologyStatistics == string("Light") || topologyStatistics== string("Heavy")) {
		activeNodesCount->record(ann);
		activeEdgesCount->record(anl);
	}

	// expensive statistics
	if (topologyStatistics== string("Heavy")) {
		partitionCount->record(pc);
	}

}

NetworkManager* getNetworkManager() {
	if (mgr == NULL) {
		cModuleType* type = cModuleType::get("routingsim.foundation.NetworkManager");
		cModule* mod = simulation.getContextModule();
		while(mod->getParentModule()!=NULL) mod = mod->getParentModule();
		mgr = dynamic_cast<NetworkManager*>(type->create( "NetworkManager",mod ));
		if (mgr == NULL)
			opp_error("Initialization of NetworkManager failed");
	}
	return mgr;
}

} //namespace
