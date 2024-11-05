#include "RegularGrid.h"
#include "clog.h"
#include "MobilityNode.h"
#include "simkerneldefs.h"
#include <string>
#include <vector>

namespace routingsim {

void RegularGrid::insert(MobilityNode* node) {
    root_cell->insert(node);
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
    north_west_coord = a;
    south_east_coord = b;
    this->depth = depth;

    Coord2D middle = (a + b) / 2.0;

    if (depth > 0) {
        // Recursively create inner cells
        north_west = new GridCell(a, middle, depth-1);
        north_east = new GridCell(Coord2D(middle.x, a.y), Coord2D(b.x, middle.y), depth-1);
        south_west = new GridCell(Coord2D(a.x, middle.y), Coord2D(middle.x, b.y), depth-1);
        south_east = new GridCell(middle, b, depth-1);
    } else {
        // Leaf cell
        nodes = std::vector<MobilityNode*>();
    }
}

bool GridCell::isInside(Coord2D point) {
    return north_west_coord.x <= point.x
        && point.x <= south_east_coord.x
        && south_east_coord.y <= point.y
        && point.y <= north_west_coord.y;
}

void GridCell::insert(MobilityNode* node) {
    Coord2D pos = node->getMobilityModel()->getPosition();
    if (depth == 0) {
        // No more child nodes
        ASSERT(isInside(pos));
        EV_DEBUG << "Insert node " << pos.toString() << " into cell " << this->north_west_coord.toString() << " " << this->south_east_coord.toString() << endl;
        nodes.emplace_back(node);
        return;
    }

    // Find correct child cell
    if (north_west->isInside(pos)) {north_west->insert(node); return;}
    if (north_east->isInside(pos)) {north_east->insert(node); return;}
    if (south_east->isInside(pos)) {south_east->insert(node); return;}
    if (south_west->isInside(pos)) {south_west->insert(node); return;}

    ASSERT(false); // We only reach this point if the given nodes is outside of this cell
}

std::string GridCell::toString() {
    std::string repr = "";
    for (int i = 0; i < 8 - depth; i++) {
        repr += "  ";
    }
    repr += std::to_string(depth) + " " + north_west_coord.toString()  + " " + south_east_coord.toString();
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
