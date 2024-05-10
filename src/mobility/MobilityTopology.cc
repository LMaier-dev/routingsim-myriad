
#include "MobilityTopology.h"

namespace routingsim {

MobilityTopology::MobilityTopology(cModule *networkManager) {
  this->networkManager = networkManager;
  this->numNodes = networkManager->par("mobilityNodes").intValue();
  this->rng =
      networkManager->getRNG(networkManager->par("mobilityRNG").intValue());
  EV_DEBUG << "MobilityTopology created" << endl;
}

// TODO: par
void MobilityTopology::initialize() {
  createTopology();
}

void MobilityTopology::setDatarate(MobilityNode *fromNode, MobilityNode *toNode,
                                   double datarate) {
  fromNode->setLinkDatarate(toNode, "toNeighbors$o", datarate);
}

void MobilityTopology::createTopology() {
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

    // jump over disabled nodes
    if ((*BGLtopology)[*vi].disabled) {
      // EV << "IGNORING NODE " << *vi << endl;
      continue;
    }
    std::stringstream node_name;
    node_name << "MobilNode_" << (nodeid_t)*vi;
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

  for (cModule *nodeMod : *mobilityNodeMods) {
    nodeMod->callInitialize();
    nodeMod->scheduleStart(simTime());
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
