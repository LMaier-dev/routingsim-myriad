
#include "MobilityDynamics.h"
#include "NetworkManager.h"

namespace routingsim {

Define_Module(MobilityDynamics);

void MobilityDynamics::initialize(int stage) {
  if (stage == 0) {
    updateInterval = par("updateInterval").doubleValue();
  } else if (stage == 1) {
    scheduleAt(simTime() + updateInterval * 1000,
               const_cast<cMessage *>(mobilityUpdateMsg));
  }

}
void MobilityDynamics::handleMessage(cMessage *msg) {
  if (msg->getKind() == MOBILITY_UPDATE) {
    updateMobility();
  }

  scheduleAt(simTime() + updateInterval * 1000,
             const_cast<cMessage *>(mobilityUpdateMsg));
}

void MobilityDynamics::updateMobility() {
  for (cModule *node_wrapper : *mobilityTopology->getNodeMods()) {
    updateMobilityNode(node_wrapper);
  }
}
void MobilityDynamics::updateMobilityNode(cModule *node_wrapper) {
  for (cModule *to_node : *mobilityTopology->getNodeMods()) {
    updateConnection(node_wrapper, to_node);
  }
}
void MobilityDynamics::updateConnection(cModule *fromNodeWrapper,
                                        cModule *toNodeWrapper) {
  MobilityNode *fromNode = check_and_cast<MobilityNode*>(fromNodeWrapper->getSubmodule("node"));
  MobilityNode *toNode = check_and_cast<MobilityNode*>(toNodeWrapper->getSubmodule("node"));
  if (fromNode->getId() == toNode->getId()) return;

  double distance = fromNode->getMobilityModel()->getPosition().distance(toNode->getMobilityModel()->getPosition());
  double radiusFromNode = fromNode->getAntennaModel()->getSignalPower();
  if (distance <= radiusFromNode) {
    double newBandwidth = fromNode->getAntennaModel()->pathloss(distance);
    mobilityTopology->setDatarate(fromNode, toNode, newBandwidth);
    mobilityTopology->enableLink(fromNode, toNode);

  } else {
    mobilityTopology->disableLink(fromNode, toNode);
  }
}
} // namespace routingsim