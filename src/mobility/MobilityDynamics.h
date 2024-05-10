
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
  void updateConnection(cModule *fromNode, cModule *toNode) const;
  void subscribeToMobilityUpdateSignals();
  std::vector<cIListener *> *mobilityListeners = new std::vector<cIListener *>();;

public:
  void initialize(int stage) override;
  void handleMessage(cMessage *msg) override;
  void updateMobility();
  void updateMobilityNode(cModule *node);
  MobilityTopology *mobilityTopology = nullptr;
};

class MobilityListener: public cListener {
public:
  MobilityDynamics *mobilityDynamics = nullptr;

  explicit MobilityListener(MobilityDynamics *mobilityDynamics) {
    this->mobilityDynamics = mobilityDynamics;
  }

  void receiveSignal(cComponent *src, simsignal_t id,
                     bool value, cObject *details) override {
    MobilityBase *nodeMobility = check_and_cast<MobilityBase *>(src);
    cModule *nodeWrapper = nodeMobility->getParentModule();
    mobilityDynamics->updateMobilityNode(nodeWrapper);
  }
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
