#include "RegularGrid.h"
#include "clog.h"
#include "MobilityNode.h"
#include <vector>

namespace routingsim {

void RegularGrid::insert(MobilityNode* node) {
    // TODO implement
    EV_DEBUG << "Insert Node " << node->getNodeId() << " into grid " << this << endl;
    nodes.emplace_back(node);
}

void RegularGrid::update(MobilityNode* node, Coord2D oldPosition) {
    EV_DEBUG << "Update Node " << node->getNodeId() << "from " << oldPosition.toString() << " to " << node->getMobilityModel()->getPosition().toString() << endl;
    // TODO implement
}

std::vector<MobilityNode*> RegularGrid::getNodesNear(MobilityNode* node) {
    EV_DEBUG << "Return Nodes near " << node->getNodeId() << endl;
    // TODO implement
    return nodes;
}

}
