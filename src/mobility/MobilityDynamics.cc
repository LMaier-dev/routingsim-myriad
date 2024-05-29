
#include "MobilityDynamics.h"
#include "NetworkManager.h"

namespace routingsim {

Define_Module(MobilityDynamics);

void MobilityDynamics::initialize(int stage) {
  if (stage == 1)  {
    scheduleMobilityUpdate();
  }
}

void MobilityDynamics::handleMessage(cMessage *msg) {
  if (msg->getKind() == MOBILITY_UPDATE) {
    updateMobility();
  }
  scheduleMobilityUpdate();
}

void MobilityDynamics::updateMobility() {
  for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
    auto *node = check_and_cast<MobilityNode*>(node_wrapper->getSubmodule("node"));
    updateMobilityNode(node);
  }
}
void MobilityDynamics::updateMobilityNode(MobilityNode *node) {
  EV_TRACE << "Update MobilityNode " << node->getNodeId() << " position from x:" << node->getMobilityModel()->getPosition().x << ", y:" << node->getMobilityModel()->getPosition().y << endl;
  node->getMobilityModel()->updatePosition();
  EV_TRACE << "Updated MobilityNode " << node->getNodeId() << " position to x:" << node->getMobilityModel()->getPosition().x << ", y:" << node->getMobilityModel()->getPosition().y << endl;
  for (cModule *to_node_wrapper : mobilityTopology->getNodeMods()) {
    auto *to_node = check_and_cast<MobilityNode*>(to_node_wrapper->getSubmodule("node"));
    updateConnection(node, to_node);
  }
}
void MobilityDynamics::updateConnection(MobilityNode *fromNode,
                                        MobilityNode *toNode) const {
  if (fromNode->getNodeId() == toNode->getNodeId()) return;
  double distance = fromNode->getMobilityModel()->getPosition().distance(toNode->getMobilityModel()->getPosition());
  double rangeFromNode = fromNode->getAntennaModel()->getRange();
  if (distance <= rangeFromNode) {
    double newDatarate = fromNode->getAntennaModel()->calcDatarate(distance);
    if (!mobilityTopology->isConnected(fromNode->getNodeId(), toNode->getNodeId())) {
      EV_TRACE << "Create link from node " << fromNode->getNodeId() << " to node " << toNode->getNodeId() << " with datarate " << newDatarate << "bps" << endl;
      mobilityTopology->createLink(fromNode, toNode);
    }
    mobilityTopology->setDatarate(fromNode, toNode, newDatarate);

  } else {
    if (mobilityTopology->isConnected(fromNode->getNodeId(), toNode->getNodeId())) {
      EV_TRACE << "Delete link from node " << fromNode->getNodeId() << " to node " << toNode->getNodeId() << endl;
      mobilityTopology->deleteLink(fromNode, toNode);
    }
  }
}
void MobilityDynamics::scheduleMobilityUpdate() {
  scheduleAt(simTime() + updateInterval, const_cast<cMessage *>(mobilityUpdateMsg));
}
} // namespace routingsim