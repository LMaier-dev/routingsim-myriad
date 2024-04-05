//
// Created by s_schaut on 27.03.24.
//

#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include "Environment.h"
#include "MobilityBase.h"
#include "NodeMobility.h"
#include "RandomWaypointMobility.h"
#include <omnetpp.h>

namespace routingsim {
class MobilityTopology {
public:
  /**
   * @param rng The random number generator to use for mobility.
   */
  MobilityTopology(omnetpp::cRNG *rng);

  void initialize();

private:
  omnetpp::cRNG *rng;
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
