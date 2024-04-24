
#include "MobilityNode.h"

namespace routingsim {

Define_Module(MobilityNode);

void MobilityNode::handleMessage(cMessage *msg) {
  Node::handleMessage(msg);
}
void MobilityNode::initialize() {
  Node::initialize();
  EV << par("mobilityModel").stdstringValue() << endl;
  const std::string mobilityType = par("mobilityModel").stdstringValue();
  if (mobilityType == "RandomWaypoint") {
    this->mobility_model = new RandomWaypointMobility();
  } else {
    throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
  }
  this->antenna_model = new IsotropicAntenna();
  this->mobility_model->initialize();
}

} // namespace routingsim