
#include "MobilityTopology.h"
#include "Environment.cpp"
#include "MobilityNode.cpp"
#include "RandomWaypointMobility.cpp"

namespace routingsim {

MobilityTopology::MobilityTopology(cRNG *rng) {
  this->rng = rng;
  createTopology();
}

void MobilityTopology::initialize(int stage) {
  for (MobilityNode &node : *mobilityNodes) {
    node.initialize(stage);
  }
}

void MobilityTopology::createTopology() {
  const int numNodes = par("mobilityNodes").intValue();
  this->mobilityNodes = new std::vector<MobilityNode>();
  for (int i = 0; i < numNodes; i++) {
    MobilityNode node = MobilityNode();
    this->mobilityNodes->push_back(node);
  }
}

} // namespace routingsim
