//
// Created by s_schaut on 06.04.24.
//

#include "MobilityDynamics.h"

namespace routingsim {

enum { MOBILITY_EVENT = 0 };

MobilityDynamics::MobilityDynamics(MobilityTopology *mobilityTopology) {
  updateInterval = par("updateInterval").doubleValue();
  mobilityUpdateMsg = new cMessage("MobilityEvent", MOBILITY_EVENT);
  this->mobilityTopology = mobilityTopology;
}

void MobilityDynamics::initialize(int stage) {
  cComponent::initialize(stage);
  if (mobilityTopology->getNodes() == nullptr)
    scheduleAt(simTime(), const_cast<cMessage *>(mobilityUpdateMsg));
}
void MobilityDynamics::handleMessage(cMessage *msg) {
  cSimpleModule::handleMessage(msg);

  if (msg->getKind() == MOBILITY_EVENT) {
    updateMobility();
  }

  scheduleAt(simTime() + updateInterval * 1000,
             const_cast<cMessage *>(mobilityUpdateMsg));
}

void MobilityDynamics::updateMobility() {
  for (MobilityNode *node : *mobilityTopology->getNodes()) {
    updateMobilityNode(node);
  }
}
void MobilityDynamics::updateMobilityNode(MobilityNode *node) {
  for (MobilityNode *to_node : *mobilityTopology->getNodes()) {
    updateConnection(node, to_node);
  }
}
void MobilityDynamics::updateConnection(MobilityNode *fromNode,
                                        MobilityNode *toNode) {
  if (fromNode->getId() == toNode->getId()) return;

  double distance = fromNode->getMobilityModel()->getPosition().distance(toNode->getMobilityModel()->getPosition());
  double radiusFromNode = fromNode->getAntennaModel()->getRadius();
  if (distance <= radiusFromNode) {
    double newBandwidth = fromNode->getAntennaModel()->pathloss(distance);
    mobilityTopology->setDatarate(fromNode, toNode, newBandwidth);
    mobilityTopology->enableLink(fromNode, toNode);

  } else {
    mobilityTopology->disableLink(fromNode, toNode);
  }
}
} // namespace routingsim