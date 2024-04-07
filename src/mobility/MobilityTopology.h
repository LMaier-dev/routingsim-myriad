
#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include <omnetpp.h>
#include "MobilityNode.h"

namespace routingsim {
class MobilityTopology {
public:
  /**
   * @param rng The random number generator to use for mobility.
   */
  explicit MobilityTopology(omnetpp::cRNG *rng);
  void initialize(int stage);
  vector<MobilityNode*> *getNodes() { return mobilityNodes; }
  void enableLink(MobilityNode *fromNode, MobilityNode *toNode);
  void disableLink(MobilityNode *fromNode, MobilityNode *toNode);
  void setDatarate(MobilityNode *fromNode, MobilityNode *toNode, double datarate);

private:
  omnetpp::cRNG *rng;
  vector<MobilityNode*> *mobilityNodes;
  void createTopology();
};
} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
