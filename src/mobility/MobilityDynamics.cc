#include "MobilityDynamics.h"

namespace routingsim {

Define_Module(MobilityDynamics);

void
MobilityDynamics::preDelete(cComponent *root)
{
  cancelAndDelete(mobilityUpdateMsg);
}


MobilityDynamics::~MobilityDynamics() {
  delete mobilityNodeAvgDegree;
  delete mobilityNodeAvgDatarate;
  delete mobilityNodeAvgDistance;
}


void
MobilityDynamics::initialize(int stage) {
  if (stage == 1) {
    scheduleMobilityUpdate();
    mobilityNodeAvgDegree = new cOutVector("mobilityNodeAvgDegree");
    mobilityNodeAvgDatarate = new cOutVector("mobilityNodeAvgDatarate");
    mobilityNodeAvgDistance = new cOutVector("mobilityNodeAvgDistance");
    addStatisticSample(this);
  }
}


void
MobilityDynamics::handleMessage(cMessage *msg) {
  if (msg->getKind() == MOBILITY_UPDATE) {
    updateMobility();
  }
  scheduleMobilityUpdate();
}


void MobilityDynamics::updateMobility() {
  for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
    auto *node =
        check_and_cast<MobilityNode *>(node_wrapper->getSubmodule("node"));
    updateMobilityNode(node);
  }
}


void
MobilityDynamics::updateMobilityNode(MobilityNode *node) {
  EV_TRACE << "Update MobilityNode " << node->getNodeId()
           << " position from x:" << node->getMobilityModel()->getPosition().x
           << ", y:" << node->getMobilityModel()->getPosition().y << endl;
  node->getMobilityModel()->updatePosition();
  EV_TRACE << "Updated MobilityNode " << node->getNodeId()
           << " position to x:" << node->getMobilityModel()->getPosition().x
           << ", y:" << node->getMobilityModel()->getPosition().y << endl;
  for (cModule *to_node_wrapper : mobilityTopology->getNodeMods()) {
    auto *to_node =
        check_and_cast<MobilityNode *>(to_node_wrapper->getSubmodule("node"));
    updateConnection(node, to_node);
  }
}


void
MobilityDynamics::updateConnection(MobilityNode *fromNode,
                                        MobilityNode *toNode) const {
  if (fromNode->getNodeId() == toNode->getNodeId())
    return;
  double distance = fromNode->getMobilityModel()->getPosition().distance(
      toNode->getMobilityModel()->getPosition());
  double rangeFromNode = fromNode->getAntennaModel()->getRange();
  double rangeToNode = toNode->getAntennaModel()->getRange();
  if (distance <= std::min(rangeFromNode, rangeToNode)) {
    double newDatarate = fromNode->getAntennaModel()->calcDatarate(distance);
    if (!mobilityTopology->isConnected(fromNode->getNodeId(),
                                       toNode->getNodeId())) {
      EV_TRACE << "Create link from node " << fromNode->getNodeId()
               << " to node " << toNode->getNodeId() << " with datarate "
               << newDatarate << "bps" << endl;
      mobilityTopology->createLink(fromNode, toNode);
    }
    mobilityTopology->setDatarate(fromNode, toNode, newDatarate);

  } else {
    if (mobilityTopology->isConnected(fromNode->getNodeId(),
                                      toNode->getNodeId())) {
      EV_TRACE << "Delete link from node " << fromNode->getNodeId()
               << " to node " << toNode->getNodeId() << endl;
      mobilityTopology->deleteLink(fromNode, toNode);
    }
  }
}


void MobilityDynamics::scheduleMobilityUpdate() {
  scheduleAt(simTime() + updateInterval, mobilityUpdateMsg);
}


void MobilityDynamics::sampleStatistics(Statistics &stats) {
  string topologyStatistics = par("topologyStatistics").stdstringValue();
  auto numActiveLinks = mobilityTopology->getActiveNumberLinks();
  auto numActiveNodes = mobilityTopology->getActiveNumberNodes();
  ASSERT(numActiveNodes > 0);
  ASSERT(numActiveLinks > 0);
  mobilityNodeAvgDegree->record((2 * numActiveLinks) / numActiveNodes);

  // mobilityNodeAvgDatarate
  double avgDatarate = 0;
  if (topologyStatistics == "Heavy") {
    for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
      auto *fromNode =
          check_and_cast<MobilityNode *>(node_wrapper->getSubmodule("node"));
      for (cModule *to_node_wrapper : mobilityTopology->getNodeMods()) {
        auto *toNode = check_and_cast<MobilityNode *>(
            to_node_wrapper->getSubmodule("node"));
        if (mobilityTopology->isConnected(fromNode->getNodeId(),
                                          toNode->getNodeId())) {
          avgDatarate += (mobilityTopology->getDatarate(fromNode, toNode) /
                          numActiveLinks);
        }
      }
    }
  }
  mobilityNodeAvgDatarate->record(avgDatarate);

  // mobilityNodeAvgDistance
  double avgDistance = 0.0;
  double numDistance = 1.0;
  if (topologyStatistics == "Heavy" and numActiveNodes > 1) {
    numDistance = (numActiveNodes * (numActiveNodes - 1)) / 2.0;
    auto nodeMods = mobilityTopology->getNodeMods();
    for (unsigned int i = 0; i < nodeMods.size(); i++) {
      auto *fromNode =
          check_and_cast<MobilityNode *>(nodeMods.at(i)->getSubmodule("node"));
      for (unsigned int j = (i + 1); j < nodeMods.size(); j++) {
        auto *toNode = check_and_cast<MobilityNode *>(
            nodeMods.at(j)->getSubmodule("node"));
        double distance = fromNode->getMobilityModel()->getPosition().distance(
            toNode->getMobilityModel()->getPosition());
        avgDistance += distance;
      }
    }
  }
  ASSERT(numDistance != 0);
  mobilityNodeAvgDistance->record(avgDistance / numDistance);
}

} // namespace routingsim
