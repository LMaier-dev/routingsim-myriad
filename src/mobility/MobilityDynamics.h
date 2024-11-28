//
// This module is responsible for updating the position of and the links between the nodes
// MobilityDynamics sends itself a self-message periodically to update the topology.
// Per update it calls the move() method of each mobile node first to update their positions.
// After all node positions have been updated, the links of each node to the others are updated according to the new position.
// This may add or remove links. To do this, the createLink(nodeid\_t, nodeid\_t) or deleteLink(nodeid\_t) method is called.
// The data rate of the active links is also updated.
// For this purpose, the method calcDatarate(double distance) is called, which calculates the data rate between two nodes depending on the distance.
//

#ifndef ROUTINGSIM_MOBILITYDYNAMICS_H
#define ROUTINGSIM_MOBILITYDYNAMICS_H

#include "MobilityTopology.h"
#include "StatisticSample.h"

namespace routingsim {


class MobilityDynamics : public cSimpleModule, public StatisticSample {
private:
  enum mobilityDynamicsMsgType {
    MOBILITY_UPDATE= 0,
  };

  // The self message periodically sent to itself
  cMessage *mobilityUpdateMsg= nullptr;

  // the update interval that controls the time between periodic updates
  double updateInterval = 0.1;

  // ProximityGrid
  RegularGrid* grid= nullptr;

  // updates the position and connection between two specific MobilityNodes
  void updateConnection(MobilityNode *fromNode, MobilityNode *toNode) const;

  // schedule an update cycle after updateInterval has passed
  void scheduleMobilityUpdate();

  // statistics
  cOutVector *mobilityNodeAvgDegree= nullptr;
  cOutVector *mobilityNodeAvgDatarate= nullptr;
  cOutVector *mobilityNodeAvgDistance= nullptr;
  cHistogram *accelerationQuotient= nullptr;

protected:
  void sampleStatistics(Statistics &stats) override;

public:
  MobilityDynamics()
      : mobilityUpdateMsg(new cMessage("MobilityEvent", MOBILITY_UPDATE)) {};

  ~MobilityDynamics();
  void preDelete(cComponent *root) override;

  void finish() override;

  MobilityTopology *mobilityTopology = nullptr;
  void initialize(int stage) override;


  // hook for the self messages
  void handleMessage(cMessage *msg) override;

  // starts an update cycle for all MobilityNodes
  void updateMobility();

  // starts an update cycle for a specific MobilityNode
  void updateMobilityNode(MobilityNode *node);
};


} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYDYNAMICS_H
