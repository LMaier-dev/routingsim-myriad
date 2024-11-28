#include "MobilityDynamics.h"
#include "BaseTypes.h"
#include "MobilityTopology.h"
#include "checkandcast.h"
#include "csimulation.h"
#include "mobility/MobilityNode.h"
#include <unordered_set>

namespace routingsim {

Define_Module(MobilityDynamics);

void
MobilityDynamics::preDelete(cComponent *root)
{
  cancelAndDelete(mobilityUpdateMsg);
  delete mobilityNodeAvgDegree;
  delete mobilityNodeAvgDatarate;
  delete mobilityNodeAvgDistance;
  delete accelerationQuotient;
}


MobilityDynamics::~MobilityDynamics() {
}

void MobilityDynamics::finish() {
  if (accelerationQuotient) {
    // Record Acceleration Statistic
    accelerationQuotient->record();
  }
}

void
MobilityDynamics::initialize(int stage) {
  if (stage == 1) {
    scheduleMobilityUpdate();
    mobilityNodeAvgDegree = new cOutVector("mobilityNodeAvgDegree");
    take(mobilityNodeAvgDegree);
    mobilityNodeAvgDatarate = new cOutVector("mobilityNodeAvgDatarate");
    take(mobilityNodeAvgDatarate);
    mobilityNodeAvgDistance = new cOutVector("mobilityNodeAvgDistance");
    take(mobilityNodeAvgDistance);
    addStatisticSample(this);

    if (par("useGrid").boolValue()) {
      grid = new RegularGrid(par("borderX"), par("borderY"), par("gridDepth"));
      mobilityTopology->initializeProximityGrid(grid);
      accelerationQuotient = new cHistogram("accelerationQuotient");
      take(accelerationQuotient);
    }
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
  // Map Node to set of oldNodes (nodes that are in range before the position update)
  std::map<MobilityNode*, std::unordered_set<MobilityNode*>> oldNodesMapping = std::map<MobilityNode*, std::unordered_set<MobilityNode*>>();
  if (grid) {
    for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
      auto *node =
          check_and_cast<MobilityNode *>(node_wrapper->getSubmodule("node"));
      oldNodesMapping[node] = grid->getNodesNear(node);
    }
  }

  // first update all positions
  for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
    auto *node =
        check_and_cast<MobilityNode *>(node_wrapper->getSubmodule("node"));
    Coord2D oldPosition = node->getMobilityModel()->getPosition();
    updateMobilityNode(node);
    if (grid)
      grid->update(node, oldPosition, simTime().dbl());
  }

  // now update connectivity for all nodes in a second step to avoid asymmetry
  for (cModule *node_wrapper : mobilityTopology->getNodeMods()) {
    auto *from_node =
      check_and_cast<MobilityNode *>(node_wrapper->getSubmodule("node"));

    if (grid) {
      // Only update near nodes
      std::unordered_set<MobilityNode*> nearNodes = grid->getNodesNear(from_node);
      for (MobilityNode *to_node : nearNodes) {
        // Calculate set difference of oldNodes\nearNodes
        oldNodesMapping[from_node].erase(to_node);

        if (from_node != to_node) {
          int i =grid->getNodesNear(from_node).size();
          int j =mobilityTopology->getNodeMods().size();
          accelerationQuotient->collect(i/(double)j);

          updateConnection(from_node, to_node);
          updateConnection(to_node, from_node);
        }
      }
    } else {
      // Update all nodes
      for (cModule *to_node_wrapper : mobilityTopology->getNodeMods()) {
        auto *to_node =
          check_and_cast<MobilityNode *>(to_node_wrapper->getSubmodule("node"));

        if (from_node != to_node) {
          updateConnection(from_node, to_node);
        }
      }
    }

    if (grid) {
      // Update removed nodes
      for (MobilityNode* old_node : oldNodesMapping[from_node]) {
        if (from_node != old_node) {
          updateConnection(from_node, old_node);
          updateConnection(old_node, from_node);
        }
      }
    }
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
  // both can communicate if their distance is covered also by the smaller range
  if (distance <= std::min(rangeFromNode, rangeToNode)) {
    double newDatarate = fromNode->getAntennaModel()->calcDatarate(distance);

    if (newDatarate > 0) {
      if (not mobilityTopology->isConnected(fromNode->getNodeId(),
					    toNode->getNodeId())) {
      	EV_TRACE << "Create link from node " << fromNode->getNodeId()
      		 << " to node " << toNode->getNodeId() << " with datarate "
      		 << newDatarate << "bps" << endl;
	      mobilityTopology->createLink(fromNode, toNode);
      }
      mobilityTopology->setDatarate(fromNode, toNode, newDatarate);
      return;
    }
  }

  // either not in range or newDatarate == 0
  if (mobilityTopology->isConnected(fromNode->getNodeId(),
				    toNode->getNodeId())) {
    EV_TRACE << "Delete link from node " << fromNode->getNodeId()
	     << " to node " << toNode->getNodeId() << endl;
    mobilityTopology->deleteLink(fromNode, toNode);
  }
}


void MobilityDynamics::scheduleMobilityUpdate() {
  scheduleAt(simTime() + updateInterval, mobilityUpdateMsg);
}


void MobilityDynamics::sampleStatistics(Statistics &stats) {
  string topologyStatistics = par("topologyStatistics").stdstringValue();
  auto numActiveLinks = mobilityTopology->getActiveNumberLinks();
  auto numActiveNodes = mobilityTopology->getActiveNumberNodes();

  if (numActiveNodes > 0)
    mobilityNodeAvgDegree->record((2.0 * numActiveLinks) / (double) numActiveNodes);
  else
    mobilityNodeAvgDegree->record(0);

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
	  if (numActiveLinks > 0)
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

  if (numDistance > 0)
    mobilityNodeAvgDistance->record(avgDistance / numDistance);
  else
    mobilityNodeAvgDistance->record(0);
}

} // namespace routingsim
