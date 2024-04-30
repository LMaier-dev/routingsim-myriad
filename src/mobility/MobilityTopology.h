
#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include "MobilityNode.h"
#include "../foundation/NetworkTopology.h"

namespace routingsim {

class MobilityTopology: public NetworkTopology {
public:
  void initialize();
  /**
   * @param rng The random number generator to use for mobility.
   */
  explicit MobilityTopology(cModule *networkManager, cRNG *rng, int numNodes);
  void setDatarate(MobilityNode *fromNode, MobilityNode *toNode, double datarate);
  std::vector<cModule*> getNodeMods() const { return *mobilityNodeMods; }
  std::vector<cModule*> *mobilityNodeMods = new std::vector<cModule *>();

private:
  cRNG *rng;
  cModule *networkManager = nullptr;

  int numNodes = 0;

  void createTopology();
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
