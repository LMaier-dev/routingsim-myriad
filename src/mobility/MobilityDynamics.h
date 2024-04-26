
#ifndef ROUTINGSIM_MOBILITYDYNAMICS_H
#define ROUTINGSIM_MOBILITYDYNAMICS_H

#include "MobilityTopology.h"

namespace routingsim {

enum {
  MOBILITY_UPDATE,
};

class MobilityDynamics : public cSimpleModule {
private:
  const cMessage *mobilityUpdateMsg =
      new cMessage("MobilityEvent", MOBILITY_UPDATE);
  double updateInterval = 0.1;
  void updateMobility();
  void updateMobilityNode(cModule *node);
  void updateConnection(cModule *fromNode, cModule *toNode);

public:
  void initialize(int stage) override;
  void handleMessage(cMessage *msg) override;
  MobilityTopology *mobilityTopology = nullptr;
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
