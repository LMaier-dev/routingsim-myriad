#include "MobilityTopology.h"
#include "cmodule.h"
#include <format>
#include <string>

namespace routingsim {

MobilityTopology::MobilityTopology(cModule *networkManager)
  : networkManager(networkManager) {
  numNodes = networkManager->par("mobilityNodes").intValue();
  mobilityNodeMods= new std::vector<cModule *>(numNodes, nullptr);
  EV_DEBUG << "MobilityTopology created" << endl;
}


void MobilityTopology::initialize() {

  for (cModule *nodeMod : *mobilityNodeMods) {
    if (!nodeMod->initialized())
      nodeMod->callInitialize();
    auto *node = check_and_cast<MobilityNode *>(nodeMod->getSubmodule("node"));
    EV_DEBUG << "MobilityNode " << node->getNodeId() << " created with range "
	     << node->getAntennaModel()->getRange() << "m" << endl;
    // nodeMod->scheduleStart(simTime());
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
  EV_INFO << "Create mobility topology" << endl;
  BGLtopology = new topologyGraph(numNodes);

  for (BGLVertexIt it = vertices(*BGLtopology).first;
       it != vertices(*BGLtopology).second; it++) {
    (*BGLtopology)[*it].maxRTentries = std::numeric_limits<int>::max();
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

  for (cModule *nodeMod : *mobilityNodeMods) {
    nodeMod->finalizeParameters();
    nodeMod->buildInside();
  }
}


void MobilityTopology::deleteLink(MobilityNode *fromNode,
                                  MobilityNode *toNode) {
  if (!isConnected(fromNode->getNodeId(), toNode->getNodeId()))
    return;

  EV_TRACE << "deleteLink " << fromNode->getNodeId() << " -> " << toNode->getNodeId() << endl;

  // this disables the link in the underlying boost graph
  disableLink(fromNode->getNodeId(), toNode->getNodeId());
  std::string fromGateName = std::format("outGateFrom{}To{}", fromNode->getNodeId(), toNode->getNodeId());
  std::string toGateName = std::format("inGateFrom{}To{}", toNode->getNodeId(), fromNode->getNodeId());

  fromNode->gate(fromGateName.c_str())->disconnect();
  toNode->gate(toGateName.c_str())->disconnect();
  fromNode->deleteGate(fromGateName.c_str());
  toNode->deleteGate(toGateName.c_str());
  // TODO: the following code does not work properly and is therefore commented out
  // unconnected gates cause confusion if reused
  //if (isConnected(fromNode->getNodeId(), toNode->getNodeId())) {
  //  disableLink(fromNode->getNodeId(), toNode->getNodeId());
  //  disableLink(toNode->getNodeId(), fromNode->getNodeId());
  //  EV_TRACE << "deleted edge " << fromNode->getNodeId() << " <-> " << toNode->getNodeId() << endl;
  // }

  // TODO: track unconnected gates, that were connected once
  //remove_edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  //cGate *gateToNode = getGateBetweenNodes(fromNode, toNode);
  //cGate *reverseGate = gateToNode->getPathEndGate();
  //gateToNode->disconnect();
  //reverseGate->disconnect();
}


void MobilityTopology::createLink(MobilityNode *fromNode,
                                  MobilityNode *toNode) {
  BGLEdgeDesc e;
  bool edgeValid;
  if (isConnected(fromNode->getNodeId(), toNode->getNodeId()))
    return;

  EV_TRACE << "createLink " << fromNode->getNodeId() << " -> " << toNode->getNodeId() << endl;

  std::string fromGateName = std::format("outGateFrom{}To{}", fromNode->getNodeId(), toNode->getNodeId());
  std::string toGateName = std::format("inGateFrom{}To{}", toNode->getNodeId(), fromNode->getNodeId());

  cGate* fromGate = fromNode->addGate(fromGateName.c_str(), cGate::OUTPUT);
  cGate* toGate = toNode->addGate(toGateName.c_str(), cGate::INPUT);

  std::string channel_name = "datarateChannel";
  cDatarateChannel *datarateChannel = cDatarateChannel::create(channel_name.c_str());
  datarateChannel->setDatarate(0);

  fromGate->connectTo(toGate, datarateChannel);

  tie(e, edgeValid) = add_edge(fromNode->getNodeId(), toNode->getNodeId(), *BGLtopology);
  if (edgeValid) {
    (*BGLtopology)[e].active = false;
    (*BGLtopology)[e].weight = 1.0;
  }
  enableLink(fromNode->getNodeId(), toNode->getNodeId());

  // only enable link, when nodes can communicate in both directions
  //if (edgeExists(toNode->getNodeId(), fromNode->getNodeId())) {
  //  enableLink(fromNode->getNodeId(), toNode->getNodeId());
  //  enableLink(toNode->getNodeId(), fromNode->getNodeId());
  //  EV_TRACE << "added edge " << fromNode->getNodeId() << " <-> " << toNode->getNodeId() << endl;
  //}
}

} // namespace routingsim
