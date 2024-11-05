#include "RegularGrid.h"
#include "clog.h"
#include <vector>

namespace routingsim {

void RegularGrid::insert(MobilityNode* node) {
    // TODO implement
    EV_DEBUG << "Insert Node " << node->getNodeId() << " into grid " << this << endl;
    nodes.emplace_back(node);
}

std::vector<MobilityNode*> RegularGrid::getNodesNear(MobilityNode* node) {
    EV_DEBUG << "Return Nodes near " << node->getNodeId() << endl;
    // TODO implement
    return nodes;
}

}
