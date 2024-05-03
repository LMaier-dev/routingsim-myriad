
#include "MobilityNode.h"

namespace routingsim {

Define_Module(MobilityNode);

void MobilityNode::initialize() {
  Node::initialize();
}

void MobilityNode::doBuildInside() {
  Node::doBuildInside();
  std::string mobilityModelName, antennaModelName;
  cModuleType *mobilityModelType, *antennaModelType;
  const std::string mobilityType = par("mobilityModel").stdstringValue();
  if (mobilityType == "RandomWaypoint") {
    mobilityModelName = "RWP_" + std::to_string(id);
    mobilityModelType = cModuleType::get("routingsim.mobility.RandomWaypointMobility");
    this->mobility_model = check_and_cast<RandomWaypointMobility *>(mobilityModelType->create(mobilityModelName.c_str(), getParentModule()));

  } else {
    throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
  }

  antennaModelName = "IsotropicAntenna_" + std::to_string(id);
  antennaModelType = cModuleType::get("routingsim.mobility.IsotropicAntenna");
  this->antenna_model = check_and_cast<IsotropicAntenna *>(antennaModelType->create(antennaModelName.c_str(), getParentModule()));
}

void MobilityNode::refreshDisplay() const {
    cDisplayString& ds = getParentModule()->getDisplayString();
    ds.setTagArg("p", 0, lround(mobility_model->getPosition().x));
    ds.setTagArg("p", 1, lround(mobility_model->getPosition().y));
    ds.setTagArg("r", 0, lround(antenna_model->getRange()));
}

} // namespace routingsim