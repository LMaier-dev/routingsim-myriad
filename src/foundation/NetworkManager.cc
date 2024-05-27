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

    NetworkManager *mgr = NULL;

    NetworkManager::~NetworkManager() {
        delete topology;
        delete mobilityTopology;
        mgr = NULL;
        topology = NULL;
        mobilityTopology = NULL;

        // clean up statistics
        delete activeEdgesCount;
        delete activeNodesCount;
        delete partitionCount;
        activeEdgesCount = activeNodesCount = partitionCount = NULL;
    }

    NetworkManager::NetworkManager() : mobilityTopology(nullptr) {
        // TODO: assert that there has not been another NetworkManager instantiated already
        evaluateStatistics = true;
        activeEdgesCount = activeNodesCount = partitionCount = NULL;
    }

    void NetworkManager::initialize() {
        EV << "initializing network manager" << endl;
        // set singleton variable
        mgr = this;

        { // create mobility manager
          const bool useMobility = par("mobility").boolValue();
          if (useMobility) {
            EV << "creating mobility manager" << std::endl;
            cModule *dynamicsModule = gate("mobilityDynamics")->getPathStartGate()->getOwnerModule();
            mobilityTopology = new MobilityTopology(this);
            mobilityTopology->createTopology();
            mobilityDynamics = dynamic_cast<MobilityDynamics *>(dynamicsModule);
            mobilityDynamics->mobilityTopology = mobilityTopology;
            scheduleAt(simTime(), new cMessage{"init mobility", INIT_MOBILITY});
          }
        }

        // create network topology
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

        topology->setShortestPathCalculationMode(par("shortestPathCalculationMode").stdstringValue());
        std::cout << "shortestPathCalculationMode: " << par("shortestPathCalculationMode").stdstringValue() << endl;

        // dynamics module connected? yes-> get dynamics module!
        cGate *dynamicsGate = gate("dynamics");
        if (dynamicsGate->isPathOK()) {
            cModule *dynamicsModule =
                    dynamicsGate->getPathStartGate()->getOwnerModule();
            dynamics = dynamic_cast<NetworkDynamics *>(dynamicsModule);
            EV << "Dynamics Module provided: " << dynamics->getModuleType()->getName() << endl;
        }

        // no-> create using given name
        if (dynamics == NULL) {
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
        if (dynamics != NULL) {
            EV << "Using dynamic model: " << dynamics->getModuleType()->getName() << endl;
            dynamics->manager = this;
            dynamics->topology = topology;
        } else
            EV << "No dynamic model available." << endl;

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
            assert(di != NULL);

            evaluateStatistics = true;
            switch (di->type) {
                case DynamicsInfo::nodeFailure:
                    topology->disableNode(di->source);
                    failedNodes.insert(di->source);
                    topology->resetNode(di->source);
                    break;
                case DynamicsInfo::nodeRecovery:
                    topology->enableNode(di->source);
                    failedNodes.erase(di->source);
                    break;
                case DynamicsInfo::linkFailure:
                    // depending on bidirectional flag deactivate one or both directions
                    topology->disableLink(di->source, di->destination);
                    failedNodes.insert(di->source);
                    if (di->bidirectionalLinkFailure) {
                        topology->disableLink(di->destination, di->source);
                    }
                    break;
                case DynamicsInfo::linkRecovery:
                    // depending on bidirectional flag restore one or both directions
                    topology->enableLink(di->source, di->destination);
                    failedNodes.erase(di->source);
                    if (di->bidirectionalLinkFailure) {
                        topology->enableLink(di->destination, di->source);
                    }
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
        string topologyStatistics = par("topologyStatistics").stdstringValue();
        static unsigned int ann, anl, pc;

        if (topologyStatistics == string("None")) return;

        if (evaluateStatistics) {
            if (topologyStatistics == string("Light") || topologyStatistics == string("Heavy")) {
                ann = topology->getActiveNumberNodes() + mobilityTopology->getActiveNumberNodes();
                anl = topology->getActiveNumberLinks() + mobilityTopology->getActiveNumberLinks();
            }
            if (topologyStatistics == string("Heavy")) {
                pc = topology->getNumPartitions() + mobilityTopology->getNumPartitions();
            }
            evaluateStatistics = false;
        }

        if (topologyStatistics == string("Light") || topologyStatistics == string("Heavy")) {
            activeNodesCount->record(ann);
            activeEdgesCount->record(anl);
        }

        // expensive statistics
        if (topologyStatistics == string("Heavy")) {
            partitionCount->record(pc);
        }
    }


    NetworkManager *getNetworkManager() {
        if (mgr == NULL) {
            cModuleType *type = cModuleType::get("routingsim.foundation.NetworkManager");
            cModule *mod = getSimulation()->getContextModule();
            while (mod->getParentModule() != NULL) mod = mod->getParentModule();
            mgr = dynamic_cast<NetworkManager *>(type->create("NetworkManager", mod));
            if (mgr == NULL)
                throw cRuntimeError("Initialization of NetworkManager failed");
        }
        return mgr;
    }

} //namespace
