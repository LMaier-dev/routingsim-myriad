
#include "MobilityNode.h"

namespace routingsim {

Define_Module(MobilityNode);

void MobilityNode::handleMessage(cMessage *msg) {
  Node::handleMessage(msg);
}
void MobilityNode::initialize() {
  std::string mobilityModelName, antennaModelName;
  cModuleType *mobilityModelType, *antennaModelType;
  std::cout << "start init mobility node " << id << std::endl;
  Node::initialize();
  const std::string mobilityType = par("mobilityModel").stdstringValue();
  if (mobilityType == "RandomWaypoint") {
    mobilityModelName = "RWP_" + std::to_string(id);
    mobilityModelType = cModuleType::get("routingsim.mobility.RandomWaypointMobility");
    this->mobility_model = check_and_cast<RandomWaypointMobility *>(mobilityModelType->createScheduleInit(mobilityModelName.c_str(), this));
  } else {
    throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
  }

  antennaModelName = "IsotropicAntenna_" + std::to_string(id);
  antennaModelType = cModuleType::get("routingsim.mobility.IsotropicAntenna");
  this->antenna_model = check_and_cast<IsotropicAntenna *>(antennaModelType->createScheduleInit(antennaModelName.c_str(), this));
}

} // namespace routingsim