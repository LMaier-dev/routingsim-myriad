
#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include "Environment.h"
#include "MobilityBase.h"
#include "MobilityNode.h"
#include "RandomWaypointMobility.h"
#include <omnetpp.h>

namespace routingsim {
class MobilityTopology {
public:
  /**
   * @param rng The random number generator to use for mobility.
   */
  explicit MobilityTopology(omnetpp::cRNG *rng);
  void initialize(int stage);
  vector<MobilityNode> *getNodes() { return mobilityNodes; }

private:
  omnetpp::cRNG *rng;
  vector<MobilityNode> *mobilityNodes;
  void createTopology();
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
