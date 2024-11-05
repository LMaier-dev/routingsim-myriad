#include "RegularGrid.h"
#include "clog.h"
#include "MobilityNode.h"
#include <string>
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

GridCell::GridCell(Coord2D a, Coord2D b, int depth) {
    south_west_coord = a;
    north_east_coord = b;
    this->depth = depth;

    Coord2D middle = (a + b) / 2.0;

    if (depth > 0) {
        // Recursively create inner cells
        north_west = new GridCell(a, middle, depth-1);
        north_east = new GridCell(Coord2D(middle.x, a.y), Coord2D(b.x, middle.y), depth-1);
        south_west = new GridCell(Coord2D(a.x, middle.y), Coord2D(middle.x, b.y), depth-1);
        south_east = new GridCell(middle, b, depth-1);
    }
}

std::string GridCell::toString() {
    std::string repr = "";
    for (int i = 0; i < 8 - depth; i++) {
        repr += "  ";
    }
    repr += std::to_string(depth) + " " + south_west_coord.toString()  + " " + north_east_coord.toString();
    repr += "\n";
    if (depth > 0) {
        repr += north_west->toString() + "\n";
        repr += north_east->toString() + "\n";
        repr += south_west->toString() + "\n";
        repr += south_east->toString() + "\n";
    }

    return repr;
}

}
