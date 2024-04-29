
#include "MobilityNode.h"

namespace routingsim {

Define_Module(MobilityNode);

void MobilityNode::doBuildInside() {
  Node::doBuildInside();
  std::string mobilityModelName, antennaModelName;
  cModuleType *mobilityModelType, *antennaModelType;
  std::cout << "start init mobility node " << id << std::endl;
  const std::string mobilityType = par("mobilityModel").stdstringValue();
  if (mobilityType == "RandomWaypoint") {
    mobilityModelName = "RWP_" + std::to_string(id);
    mobilityModelType = cModuleType::get("routingsim.mobility.RandomWaypointMobility");
    this->mobility_model = check_and_cast<RandomWaypointMobility *>(mobilityModelType->create(mobilityModelName.c_str(), this));
    this->mobility_model->finalizeParameters();
    this->mobility_model->buildInside();

  } else {
    throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
  }

  antennaModelName = "IsotropicAntenna_" + std::to_string(id);
  antennaModelType = cModuleType::get("routingsim.mobility.IsotropicAntenna");
  this->antenna_model = check_and_cast<IsotropicAntenna *>(antennaModelType->create(antennaModelName.c_str(), this));
  this->antenna_model->finalizeParameters();
  this->antenna_model->buildInside();
}

void MobilityNode::initialize() {
  Node::initialize();
}

} // namespace routingsim