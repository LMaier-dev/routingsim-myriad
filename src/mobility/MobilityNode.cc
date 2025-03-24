#include "MobilityNode.h"
#include "RandomWaypointMobility.h"
#include "GaussMarkovMobility.h"
#include "IsotropicAntenna.h"

namespace routingsim {

Define_Module(MobilityNode);

void MobilityNode::initialize(int stage) {
  if (stage == 0) {
    Node::initialize(stage);
  } else
  if (stage == 1)  {
    Node::initialize(stage);

    std::string mobilityModelName, antennaModelName;
    cModuleType *mobilityModelType, *antennaModelType;
    const std::string mobilityType = par("mobilityModel").stdstringValue();
    mobilityModelName = "MobilityModel";
    if (mobilityType == "RandomWaypoint") {
      mobilityModelType = cModuleType::get("routingsim.mobility.RandomWaypointMobility");
      mobility_model = check_and_cast<RandomWaypointMobility *>(mobilityModelType->create(mobilityModelName.c_str(), getParentModule()));
      mobility_model->finalizeParameters();
      mobility_model->initialize();
    }
    else if (mobilityType == "GaussMarkov") {
      mobilityModelType = cModuleType::get("routingsim.mobility.GaussMarkovMobility");
      mobility_model = check_and_cast<GaussMarkovMobility *>(mobilityModelType->create(mobilityModelName.c_str(), getParentModule()));
      mobility_model->finalizeParameters();
      mobility_model->initialize();
    } else {
      throw new cRuntimeError("Unknown mobility model %s", mobilityType.c_str());
    }

    antennaModelName = "Antenna";
    antennaModelType = cModuleType::get("routingsim.mobility.IsotropicAntenna");
    antenna_model = check_and_cast<IsotropicAntenna *>(
        antennaModelType->create(antennaModelName.c_str(), getParentModule()));
    antenna_model->finalizeParameters();
    antenna_model->initialize();
  }

}


void MobilityNode::doBuildInside() {
}


void MobilityNode::refreshDisplay() const {
    cDisplayString& ds = getParentModule()->getDisplayString();
    ds.setTagArg("p", 0, lround(mobility_model->getPosition().x));
    ds.setTagArg("p", 1, lround(mobility_model->getPosition().y));
    ds.setTagArg("r", 0, lround(antenna_model->getRange()));
    Node::refreshDisplay();
}

} // namespace routingsim
