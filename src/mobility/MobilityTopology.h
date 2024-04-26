
#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include "MobilityNode.h"

namespace routingsim {

class MobilityTopology {
public:
  /**
   * @param rng The random number generator to use for mobility.
   */
  explicit MobilityTopology(cModule *networkManager, cRNG *rng, int numNodes);
  void initialize();
  std::vector<cModule*> *getNodes() { return mobilityNodes; }
  void enableLink(MobilityNode *fromNode, MobilityNode *toNode);
  void disableLink(MobilityNode *fromNode, MobilityNode *toNode);
  void setDatarate(MobilityNode *fromNode, MobilityNode *toNode, double datarate);

private:
  cRNG *rng;
  cModule *networkManager = nullptr;
  int numNodes = 0;
  std::vector<cModule*> *mobilityNodes{};
  void createTopology();
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
