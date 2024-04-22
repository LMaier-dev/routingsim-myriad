
#include "MobilityNode.h"

namespace routingsim {

Define_Module(MobilityNode);

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
void routingsim::MobilityNode::initialize() {
  Node::initialize();
  this->mobility_model->initialize();
}

} // namespace routingsim