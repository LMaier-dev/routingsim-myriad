
#ifndef ROUTINGSIM_MOBILITYDYNAMICS_H
#define ROUTINGSIM_MOBILITYDYNAMICS_H

#include "MobilityTopology.h"
#include "MobilityBase.h"

namespace routingsim {

enum {
  MOBILITY_UPDATE,
};

class MobilityDynamics : public cSimpleModule {
private:
  const cMessage *mobilityUpdateMsg = new cMessage("MobilityEvent", MOBILITY_UPDATE);
  double updateInterval = 0.1;
  void updateConnection(MobilityNode *fromNode, MobilityNode *toNode) const;
  void scheduleMobilityUpdate();

public:
  void initialize(int stage) override;
  void handleMessage(cMessage *msg) override;
  void updateMobility();
  void updateMobilityNode(MobilityNode *node);
  MobilityTopology *mobilityTopology = nullptr;
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
