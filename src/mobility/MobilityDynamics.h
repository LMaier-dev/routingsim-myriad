//
// Created by s_schaut on 06.04.24.
//

#ifndef ROUTINGSIM_MOBILITYDYNAMICS_H
#define ROUTINGSIM_MOBILITYDYNAMICS_H

namespace routingsim {

class MobilityDynamics : public cSimpleModule {
private:
  MobilityTopology *mobilityTopology;
  const cMessage *mobilityUpdateMsg;
  double updateInterval;
  void updateMobility();
  void updateMobilityNode(MobilityNode *node);
  void updateConnection(MobilityNode *fromNode, MobilityNode *toNode);

public:
  MobilityDynamics(MobilityTopology *mobilityTopology);
  void initialize(int stage) override;
  void handleMessage(cMessage *msg) override;
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
