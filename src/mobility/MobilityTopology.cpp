//
// Created by s_schaut on 27.03.24.
//

#include "MobilityTopology.h"
#include "Environment.cpp"
#include "NodeMobility.cpp"
#include "RandomWaypointMobility.cpp"
#include <omnetpp.h>

namespace routingsim {

MobilityTopology::MobilityTopology(omnetpp::cRNG *rng) {
  this->rng = rng;
}

void MobilityTopology::initialize() {}

} // namespace routingsim
