
#ifndef ROUTINGSIM_MOBILITYDYNAMICS_H
#define ROUTINGSIM_MOBILITYDYNAMICS_H

#include "MobilityTopology.h"

namespace routingsim {

class MobilityDynamics : public cSimpleModule {
private:
  MobilityTopology *mobilityTopology;
  const cMessage *mobilityUpdateMsg;
  double updateInterval;
  void updateMobility();
  void updateMobilityNode(cModule *node);
  void updateConnection(cModule *fromNode, cModule *toNode);

public:
  MobilityDynamics() {};
  MobilityDynamics(MobilityTopology *mobilityTopology);
  void initialize() override;
  void handleMessage(cMessage *msg) override;
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
