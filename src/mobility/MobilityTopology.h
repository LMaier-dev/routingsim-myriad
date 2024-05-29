
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
  explicit MobilityTopology(cModule *networkManager);
  void setDatarate(MobilityNode *fromNode, MobilityNode *toNode, double datarate);
  void createLink(MobilityNode *fromNode, MobilityNode *toNode);
  void deleteLink(MobilityNode *fromNode, MobilityNode *toNode);
  std::vector<cModule*> getNodeMods() const { return *mobilityNodeMods; }
  std::vector<cModule*> *mobilityNodeMods = new std::vector<cModule *>();
  void createTopology();

private:
  cModule *networkManager = nullptr;
  int numNodes = 0;
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
