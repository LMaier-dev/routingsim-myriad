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

namespace routingsim {

Define_Module(NetworkManager);


enum SelfMessageKind {
  INIT_MOBILITY,
};

NetworkManager *mgr = nullptr;

NetworkManager::~NetworkManager() {
	delete topology;
        delete mobilityTopology;
	// since mgr == this, do not delete
	mgr = nullptr;
	topology = nullptr;
	mobilityTopology = nullptr;

	edgeChangesCount = 0;
	nodeChangesCount = 0;
	// clean up statistics
	delete activeEdgesCountV;
	delete activeNodesCountV;
	delete partitionCount;
	activeEdgesCountV = activeNodesCountV = partitionCount = nullptr;
}


void NetworkManager::initialize() {
	EV << "initializing network manager" << endl;
	// set singleton variable
	mgr = this;

	useMobility = par("mobility").boolValue();
        if (useMobility) {
		// create mobility manager
		EV << "creating mobility manager" << std::endl;
		mobilityTopology = new MobilityTopology(this);
                mobilityTopology->createTopology();
		mobilityTopology->setShortestPathCalculationMode(par("shortestPathCalculationMode").stdstringValue());
		std::cout << "shortestPathCalculationMode (mobilityTopology): " << par("shortestPathCalculationMode").stdstringValue() << endl;
		cModule *dynamicsModule = gate("mobilityDynamics")->getPathStartGate()->getOwnerModule();
                mobilityDynamics = dynamic_cast<MobilityDynamics *>(dynamicsModule);
                if (mobilityDynamics != nullptr) {
                } else {
		  EV << "creating MobilityDynamics module" << endl;
		  cModuleType *type = cModuleType::get("MobilityDynamics");
		  dynamicsModule = type->create("dynamics", this);
		  dynamicsModule->finalizeParameters();
		}
                mobilityDynamics->mobilityTopology = mobilityTopology;
		scheduleAt(simTime(), new cMessage{"init mobility", INIT_MOBILITY});
        }

        // "fixed" network topology (links and nodes may still fail dynamically, but nodes do not move)
	// create fixed network topology (note this may exist in addtion to the mobilityTopology)
	topology = new NetworkTopology();

        if (par("topologyFile").stdstringValue().size() != 0)
		topology->readTopology(
			par("topologyFile").stdstringValue(),
			par("topologyFileType").stdstringValue(),
			par("useOnlyLargestConnectedComponent").boolValue(),
			par("makeLinksBidirectional").boolValue(),
			par("allowNodeIdRemapping").boolValue(),
			par("compressNodeIDs").boolValue()
			);

        if (par("topologyFile").stdstringValue().size() != 0) {
		topology->createNetwork(this);
	} else {
		// the network was built from a ned file
		topology->extractFromNetwork(this);
	}

        if (par("spathFile").stdstringValue().size() != 0) {
		if (topology->readShortestPathMatrix(par("spathFile").stdstringValue())) {
			EV_INFO << "successfully read shortest path matrix from " << par("spathFile").stdstringValue() << endl;
			// 		if (topology->compareSPM())
			// 			EV << "shortest path matrix matches" << endl;
			// 		else
			// 			EV << "shortest path matrix doesn't match!!" << endl;
		} else {
			EV_ERROR << "failure while reading shortest path matrix from " << par("spathFile").stdstringValue()
				 << endl;
		}
	}

	string topologyStatistics = par("topologyStatistics").stdstringValue();
        if (topologyStatistics == "None") {
		topologyStatisticsMode= topostatsmode_none;
        }
        else if (topologyStatistics == "Light") {
		topologyStatisticsMode= topostatsmode_light;
	}
        else if (topologyStatistics == "Heavy") {
		topologyStatisticsMode= topostatsmode_heavy;
	}

        topology->setShortestPathCalculationMode(par("shortestPathCalculationMode").stdstringValue());
	std::cout << "shortestPathCalculationMode: " << par("shortestPathCalculationMode").stdstringValue() << endl;

        // dynamics module connected? yes-> get dynamics module!
	cGate* dynamicsGate = gate("dynamics");
	if (dynamicsGate->isPathOK()) {
		cModule *dynamicsModule =
			dynamicsGate->getPathStartGate()->getOwnerModule();
		dynamics = dynamic_cast<NetworkDynamics *>(dynamicsModule);
		EV << "Dynamics Module provided: " << dynamics->getModuleType()->getName() << endl;
	}

	// no-> create using given name
	if (dynamics == nullptr) {
		// create dynamics modules
		string dynamicsModel = par("dynamicsModel");
		EV << "Creating dynamic model: " << dynamicsModel << endl;
		if (dynamicsModel != string("None")) {
			string modelName = string("routingsim.dynamics.") + dynamicsModel;
			cModuleType *type = cModuleType::get(modelName.c_str());
			// TODO: this causes problems with the qtenv in omnet++ 5.4
			// qtenv does not like it if a outside gate is connected to a submodule.
			// To avoid this, just create the dynamics module manually
			// next to the NetworkManager in the ned file.
			cModule *mod = type->create("dynamics", this);
			mod->finalizeParameters();

			// connect gates
			mod->gate("toManager")->connectTo(this->gate("dynamics"));

			// create internals, and schedule it
			mod->buildInside();
			mod->scheduleStart(simTime());

			dynamics = dynamic_cast<NetworkDynamics *>(mod);
		}
	}

	// initialize dynamics module
	if (dynamics != nullptr) {
		EV << "Using dynamic model: " << dynamics->getModuleType()->getName() << endl;
		dynamics->manager = this;
		dynamics->topology = useMobility ? mobilityTopology : topology;
	} else
		EV << "No dynamic model available." << endl;

	// statistics
	activeNodesCountV = new cOutVector("activeNodesCount");
	activeEdgesCountV = new cOutVector("activeEdgesCount");
        partitionCount = new cOutVector("partitionCount");
	addStatisticSample(this);
}


void NetworkManager::handleMessage(cMessage *msg) {
	if (msg->arrivedOn("dynamics")) {
		EV << "received dynamics message" << endl;

		// if it's a selfmessage, then we have DynamicsInfo attached
		DynamicsInfo* di =
			static_cast<DynamicsInfo *> (msg->getContextPointer());
		assert(di != nullptr);

		evaluateStatistics = true;
		switch (di->type) {
                case DynamicsInfo::nodeFailure:
			edgeChangesCount+= topology->getNodeDegree(di->source);
			topology->disableNode(di->source);
			failedNodes.insert(di->source);
                        topology->resetNode(di->source);
			nodeChangesCount++;
			break;
		case DynamicsInfo::nodeRecovery:
			topology->enableNode(di->source);
			edgeChangesCount+= topology->getNodeDegree(di->source);
			failedNodes.erase(di->source);
			nodeChangesCount++;
			break;
		case DynamicsInfo::linkFailure:
			// depending on bidirectional flag deactivate one or both directions
			topology->disableLink(di->source, di->destination);
			failedNodes.insert(di->source);
			if (di->bidirectionalLinkFailure) {
				topology->disableLink(di->destination, di->source);
			}
			edgeChangesCount++;
			break;
		case DynamicsInfo::linkRecovery:
			// depending on bidirectional flag restore one or both directions
			topology->enableLink(di->source, di->destination);
			failedNodes.erase(di->source);
			if (di->bidirectionalLinkFailure) {
				topology->enableLink(di->destination, di->source);
			}
			edgeChangesCount++;
                        break;
		case DynamicsInfo::undefined:
			EV_ERROR << "undefined event, did you forget to set DynamicsInfo::type?" << endl;
			break;
		default:
			EV_ERROR << "undefined event, did you forget to set DynamicsInfo::type?" << endl;
		}
		delete di;
	} else if (msg->isSelfMessage()) {
		switch (msg->getKind()) {
		case INIT_MOBILITY:
			mobilityTopology->initialize();
			mobilityDynamics->initialize(1);
			break;
		default:
			throw new cRuntimeError("unknown self-message kind %hd", msg->getKind());
		}
	}
	delete msg;
}

void NetworkManager::finish() {
}

void NetworkManager::sampleStatistics(Statistics &stats) {
	static unsigned int ann, anl, pc;

	if (topologyStatisticsMode == topostatsmode_none) return;

	if (evaluateStatistics) {
		if (topologyStatisticsMode == topostatsmode_light or topologyStatisticsMode == topostatsmode_heavy) {
			ann = topology->getActiveNumberNodes() + (useMobility ? mobilityTopology->getActiveNumberNodes() : 0);
			anl = topology->getActiveNumberLinks() + (useMobility ? mobilityTopology->getActiveNumberLinks() : 0);
		}
		if (topologyStatisticsMode == topostatsmode_heavy) {
			pc = topology->getNumPartitions() + mobilityTopology->getNumPartitions();
		}
		// evaluateStatistics = false;
	}

	if (topologyStatisticsMode == topostatsmode_light or topologyStatisticsMode == topostatsmode_heavy) {
		activeNodesCountV->record(ann);
                activeEdgesCountV->record(anl);
		stats.value("edgeChangesCount", edgeChangesCount);
		stats.value("nodeChangesCount", nodeChangesCount);
	}

	// expensive statistics
	if (topologyStatisticsMode == topostatsmode_heavy) {
		partitionCount->record(pc);
	}
}


NetworkManager *getNetworkManager() {
	if (mgr == nullptr) {
		cModuleType *type = cModuleType::get("routingsim.foundation.NetworkManager");
		cModule *mod = getSimulation()->getContextModule();
		while (mod->getParentModule() != nullptr) mod = mod->getParentModule();
		mgr = dynamic_cast<NetworkManager *>(type->create("NetworkManager", mod));
		if (mgr == nullptr)
			throw cRuntimeError("Initialization of NetworkManager failed");
	}
	return mgr;
}

} //namespace
