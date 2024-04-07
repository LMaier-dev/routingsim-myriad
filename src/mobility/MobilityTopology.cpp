
#include "MobilityTopology.h"
#include <omnetpp.h>

namespace routingsim {

MobilityTopology::MobilityTopology(cRNG *rng) {
  this->rng = rng;
  createTopology();
}

void MobilityTopology::initialize(int stage) {
  for (MobilityNode *node : *mobilityNodes) {
    node->initialize(stage);
  }
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
  const int numNodes = par("mobilityNodes").intValue();
  this->mobilityNodes = new std::vector<MobilityNode *>();
  for (int i = 0; i < numNodes; i++) {
    MobilityNode node = MobilityNode();
    this->mobilityNodes->push_back(&node);
  }

  cGate *fromOut, *toIn;
  cDatarateChannel *datarateChannel;
  for (int i = 0; i < numNodes; ++i) {
    fromOut = this->mobilityNodes->at(i)->getOrCreateFirstUnconnectedGate(
        "toNeighbors", 'o', false, true);
    for (int j = 0; j < numNodes; ++j) {
      if (i == j)
        continue;
      else {
        toIn = this->mobilityNodes->at(j)->getOrCreateFirstUnconnectedGate(
            "toNeighbors", 'i', false, true);
        datarateChannel = cDatarateChannel::create("channel");
        datarateChannel->setDatarate(0.0);
        fromOut->connectTo(toIn, datarateChannel);
        datarateChannel->callInitialize();
      }
    }
  }
}

} // namespace routingsim
