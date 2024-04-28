
#include "MobilityTopology.h"

namespace routingsim {

MobilityTopology::MobilityTopology(cModule *networkManager, cRNG *rng,
                                   int numNodes) {
  this->networkManager = networkManager;
  this->numNodes = numNodes;
  this->rng = rng;
  EV_DEBUG << "MobilityTopology created" << endl;
}

void MobilityTopology::initialize() { createTopology(); }

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
  this->mobilityNodes = new std::vector<cModule *>();
  cModuleType *node_wrapper_type =
      cModuleType::get("routingsim.mobility.MobilityNodeWrapper");
  for (int i = 0; i < numNodes; i++) {
    std::string node_wrapper_name = "mobilityNodeWrapper_" + std::to_string(i);
    cModule *node_wrapper =
        node_wrapper_type->create(node_wrapper_name.c_str(), networkManager);
    node_wrapper->par("nodeId") = i;
    this->mobilityNodes->push_back(node_wrapper);
  }

  cGate *fromOut, *toIn;
  cModule *fromNode, *toNode;
  cDatarateChannel *datarateChannel;
  for (int i = 0; i < numNodes; ++i) {
    for (int j = (i + 1); j < numNodes; ++j) {
      fromNode = this->mobilityNodes->at(i);
      fromOut = fromNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'o',
                                                          false, true);
      toNode = this->mobilityNodes->at(j);
      toIn = toNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'i', false,
                                                     true);
      std::string channel_name =
          "mobilityChannel_" + std::to_string(i) + "-" + std::to_string(j);
      datarateChannel = cDatarateChannel::create(channel_name.c_str());
      datarateChannel->setDatarate(0.0);
      fromOut->connectTo(toIn, datarateChannel);
    }
  }

  for (cModule *node : *mobilityNodes) {
    node->finalizeParameters();
    node->buildInside();
  }

  for (cModule *node : *mobilityNodes) {
    node->callInitialize();
    node->scheduleStart(simTime());
  }
}

} // namespace routingsim
