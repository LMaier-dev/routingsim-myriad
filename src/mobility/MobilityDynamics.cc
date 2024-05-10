
#include "MobilityDynamics.h"
#include "NetworkManager.h"

namespace routingsim {

Define_Module(MobilityDynamics);

void MobilityDynamics::initialize(int stage) {
  if (stage == 1)  {
    subscribeToMobilityUpdateSignals();
  }
}

void MobilityDynamics::subscribeToMobilityUpdateSignals() {
  for (cModule *to_node : mobilityTopology->getNodeMods()) {
    MobilityListener *listener = new MobilityListener(this);
    to_node->getSubmodule("MobilityModel")->subscribe("mobilityStateChanged", listener);
    mobilityListeners->push_back(listener);
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
  for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
    updateMobilityNode(node_wrapper);
  }
}
void MobilityDynamics::updateMobilityNode(cModule *node_wrapper) {
  for (cModule *to_node : mobilityTopology->getNodeMods()) {
    updateConnection(node_wrapper, to_node);
  }
}
void MobilityDynamics::updateConnection(cModule *fromNodeWrapper,
                                        cModule *toNodeWrapper) const {
  auto *fromNode = check_and_cast<MobilityNode*>(fromNodeWrapper->getSubmodule("node"));
  auto *toNode = check_and_cast<MobilityNode*>(toNodeWrapper->getSubmodule("node"));
  if (fromNode->getId() == toNode->getId()) return;

  double distance = fromNode->getMobilityModel()->getPosition().distance(toNode->getMobilityModel()->getPosition());
  double rangeFromNode = fromNode->getAntennaModel()->getRange();
  EV_DEBUG << "Connection update" << endl;
  EV_DEBUG << "------ from: " << fromNode->getNodeId() << endl;
  EV_DEBUG << "------ to: " << toNode->getNodeId() << endl;
  EV_DEBUG << "------ range: " << rangeFromNode << endl;
  if (distance <= rangeFromNode) {
    double newDatarate = fromNode->getAntennaModel()->calcDatarate(distance);
    EV_DEBUG << "------ new datarate: " << newDatarate << endl;
    mobilityTopology->setDatarate(fromNode, toNode, newDatarate);
    mobilityTopology->enableLink(fromNode->getNodeId(), toNode->getNodeId());

  } else {
    mobilityTopology->disableLink(fromNode->getNodeId(), toNode->getNodeId());
  }
}
} // namespace routingsim