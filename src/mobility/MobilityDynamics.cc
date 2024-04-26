
#include "MobilityDynamics.h"

namespace routingsim {

Define_Module(MobilityDynamics);

enum { MOBILITY_EVENT = 0 };

MobilityDynamics::MobilityDynamics(MobilityTopology *mobilityTopology) {
  mobilityUpdateMsg = new cMessage("MobilityEvent", MOBILITY_EVENT);
  this->mobilityTopology = mobilityTopology;
}

void MobilityDynamics::initialize() {
  cComponent::initialize();
  updateInterval = par("updateInterval").doubleValue();
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
  for (cModule *node_wrapper : *mobilityTopology->getNodes()) {
    updateMobilityNode(node_wrapper);
  }
}
void MobilityDynamics::updateMobilityNode(cModule *node_wrapper) {
  for (cModule *to_node : *mobilityTopology->getNodes()) {
    updateConnection(node_wrapper, to_node);
  }
}
void MobilityDynamics::updateConnection(cModule *fromNodeWrapper,
                                        cModule *toNodeWrapper) {
  MobilityNode *fromNode = check_and_cast<MobilityNode*>(fromNodeWrapper->getModuleByPath("node"));
  MobilityNode *toNode = check_and_cast<MobilityNode*>(toNodeWrapper->getModuleByPath("node"));
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