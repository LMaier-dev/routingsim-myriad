
#include "MobilityTopology.h"

namespace routingsim {

MobilityTopology::MobilityTopology(cModule *networkManager, cRNG *rng,
                                   int numNodes) {
  this->networkManager = networkManager;
  this->numNodes = numNodes;
  this->rng = rng;
  EV_DEBUG << "MobilityTopology created" << endl;
}

// TODO: par
void MobilityTopology::initialize() {
  NetworkTopology::readTopology("../simulations/topologies/fully_connected_n6.txt", "SELF_DEFINED", true, false, false, false);
  NetworkTopology::createNetwork(networkManager, "routingsim.mobility.MobilityNodeWrapper");
  setShortestPathCalculationMode("fullDistanceMatrix");
}

void MobilityTopology::enableLink(MobilityNode *fromNode,
                                  MobilityNode *toNode) {
  fromNode->enableLink(toNode, "toNeighbors$o");
}

void MobilityTopology::disableLink(MobilityNode *fromNode,
                                   MobilityNode *toNode) {
  fromNode->disableLink(toNode, "toNeighbors$o");
}

void MobilityTopology::setDatarate(MobilityNode *fromNode, MobilityNode *toNode,
                                   double datarate) {
  fromNode->setLinkDatarate(toNode, "toNeighbors$o", datarate);
}

void MobilityTopology::createTopology() {
  this->mobilityNodeMods = new std::vector<cModule *>();
  cModuleType *node_wrapper_type =
      cModuleType::get("routingsim.mobility.MobilityNodeWrapper");
  for (int i = 0; i < numNodes; i++) {
    std::string node_wrapper_name = "mobilityNodeWrapper_" + std::to_string(i);
    cModule *node_wrapper =
        node_wrapper_type->create(node_wrapper_name.c_str(), networkManager);
    node_wrapper->par("nodeId") = i;
    this->mobilityNodeMods->push_back(node_wrapper);
  }

  cGate *fromOut, *toIn;
  cModule *fromNode, *toNode;
  cDatarateChannel *datarateChannel;
  for (int i = 0; i < numNodes; i++) {
    for (int j = 0; j < numNodes; j++) {
      if (i == j) {
        continue;
      }

      fromNode = this->mobilityNodeMods->at(i);
      fromOut = fromNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'o',
                                                          false, true);
      toNode = this->mobilityNodeMods->at(j);
      toIn = toNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'i', false,
                                                     true);
      std::string channel_name =
          "mobilityChannel_" + std::to_string(i) + "-" + std::to_string(j);
      datarateChannel = cDatarateChannel::create(channel_name.c_str());
      datarateChannel->setDatarate(1000.0);
      fromOut->connectTo(toIn, datarateChannel);
    }
  }

  for (cModule *nodeMod : *mobilityNodeMods) {
    nodeMod->finalizeParameters();
    nodeMod->buildInside();
  }

  for (cModule *nodeMod : *mobilityNodeMods) {
    nodeMod->callInitialize();
    nodeMod->scheduleStart(simTime());
  }
}

} // namespace routingsim
