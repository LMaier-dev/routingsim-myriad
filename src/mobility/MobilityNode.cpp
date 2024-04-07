
#include "MobilityNode.h"
#include "MobilityBase.cpp"
#include "IsotropicAntenna.cpp"
#include "RandomWaypointMobility.cpp"

namespace routingsim {

MobilityNode::MobilityNode() {
  const std::string mobilityType = par("mobilityModel").stdstringValue();
  if (mobilityType == "RandomWaypoint") {
    this->mobility_model = new RandomWaypointMobility();
  } else {
    throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
  }
  this->antenna_model = new IsotropicAntenna();
}

void routingsim::MobilityNode::handleMessage(cMessage *msg) {
  Node::handleMessage(msg);
}
void routingsim::MobilityNode::initialize(int stage) {
  Node::initialize();
  this->mobility_model->initialize(stage);
  // this->antenna_model->initialize(stage);
}

} // namespace routingsim