
#include "MobilityTopology.h"

namespace routingsim {

MobilityTopology::MobilityTopology(cModule *networkManager) {
  this->networkManager = networkManager;
  this->numNodes = networkManager->par("mobilityNodes").intValue();
  EV_DEBUG << "MobilityTopology created" << endl;
}

void MobilityTopology::initialize() {
  for (cModule *nodeMod : *mobilityNodeMods) {
    if (!nodeMod->initialized())
      nodeMod->callInitialize();
    auto *node = check_and_cast<MobilityNode *>(nodeMod->getSubmodule("node"));
    EV_DEBUG << "MobilityNode " << node->getNodeId() << " created with range "
             << node->getAntennaModel()->getRange() << "m" << endl;
    nodeMod->scheduleStart(simTime());
  }
}

void MobilityTopology::setDatarate(MobilityNode *fromNode, MobilityNode *toNode,
                                   double datarate) {
  fromNode->setLinkDatarate(toNode, "toNeighbors$o", datarate);
}

double MobilityTopology::getDatarate(MobilityNode *fromNode,
                                     MobilityNode *toNode) {
  cGate *gateToNode = getGateBetweenNodes(fromNode, toNode);
  auto *channel = check_and_cast<cDatarateChannel *>(gateToNode->getChannel());
  return channel->getDatarate();
}

cGate *MobilityTopology::getGateBetweenNodes(MobilityNode *fromNode,
                                             MobilityNode *toNode) {
  const char *gatename = "toNeighbors$o";
  for (int i = 0; i < fromNode->gateSize(gatename); i++) {
    cGate *gateToNode = fromNode->gate(gatename, i);
    if (toNode == fromNode->getOtherNode(gateToNode)) {
      return gateToNode;
    }
  }

  return nullptr;
}

void MobilityTopology::createTopology() {
  EV_INFO << "Create mobility typology" << endl;
  BGLtopology = new topologyGraph(numNodes);

  for (BGLVertexIt it = vertices(*BGLtopology).first;
       it != vertices(*BGLtopology).second; it++) {
    (*BGLtopology)[*it].disabled = false;
    (*BGLtopology)[*it].active = true;
  }

  BGLVertexIt vi, ve;
  this->mobilityNodeMods = new std::vector<cModule *>();
  cModuleType *node_wrapper_type =
      cModuleType::get("routingsim.mobility.MobilityNodeWrapper");

  for (tie(vi, ve) = vertices(*BGLtopology); vi != ve; vi++) {
    std::stringstream node_name;
    node_name << "MobilityNode_" << (nodeid_t)*vi;
    cModule *node_wrapper =
        node_wrapper_type->create(node_name.str().c_str(), networkManager);
    node_wrapper->par("nodeId") = *vi;
    moduleMap[*vi] = node_wrapper;
    this->mobilityNodeMods->push_back(node_wrapper);
  }

  setShortestPathCalculationMode("fullDistanceMatrix");

  for (cModule *nodeMod : *mobilityNodeMods) {
    nodeMod->finalizeParameters();
    nodeMod->buildInside();
  }
}

void MobilityTopology::deleteLink(MobilityNode *fromNode,
                                  MobilityNode *toNode) {
  BGLEdgeDesc e;
  bool edgeValid;
  tie(e, edgeValid) =
      edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  if (!edgeValid)
    return;
  disableLink(fromNode->getNodeId(), toNode->getNodeId());
  remove_edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  cGate *gateToNode = getGateBetweenNodes(fromNode, toNode);
  gateToNode->disconnect();
}

void MobilityTopology::createLink(MobilityNode *fromNode,
                                  MobilityNode *toNode) {
  BGLEdgeDesc e;
  bool edgeValid;
  tie(e, edgeValid) =
      edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  if (edgeValid)
    return;

  cGate *fromOut = fromNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'o',
                                                             false, true);
  cGate *toIn =
      toNode->getOrCreateFirstUnconnectedGate("toNeighbors", 'i', false, true);
  std::string channel_name = "datarateChannel";
  cDatarateChannel *datarateChannel =
      cDatarateChannel::create(channel_name.c_str());
  datarateChannel->setDatarate(1000.0);
  fromOut->connectTo(toIn, datarateChannel);

  tie(e, edgeValid) =
      add_edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  if (edgeValid) {
    (*BGLtopology)[e].active = false;
    (*BGLtopology)[e].weight = 1.0;
  }
  enableLink(fromNode->getNodeId(), toNode->getNodeId());
}

} // namespace routingsim
