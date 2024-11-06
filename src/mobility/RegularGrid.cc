#include "RegularGrid.h"
#include "clog.h"
#include "MobilityNode.h"
#include "simkerneldefs.h"
#include <cassert>
#include <string>
#include <unordered_set>
#include <vector>

namespace routingsim {

void RegularGrid::insert(MobilityNode* node) {
    root_cell->insert(node);
}

void RegularGrid::update(MobilityNode* node, Coord2D oldPosition) {
    GridCell* oldCell = root_cell->findCell(oldPosition);
    GridCell* newCell = root_cell->findCell(node->getMobilityModel()->getPosition());
    if (newCell != oldCell) {
        EV_DEBUG << "Node " << node->getNodeId() << " moved from " << oldCell->printBounds() << " to " << newCell->printBounds() << endl;
        EV_DEBUG << root_cell->toString() << endl;
        oldCell->remove(node, oldPosition);
        newCell->insert(node);
        EV_DEBUG << root_cell->toString() << endl;
    }
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
    east_cell = south_cell = nullptr;

    Coord2D middle = (a + b) / 2.0;

    if (depth > 0) {
        // Recursively create inner cells
        north_west = new GridCell(a, middle, depth-1);
        north_east = new GridCell(Coord2D(middle.x, a.y), Coord2D(b.x, middle.y), depth-1);
        south_west = new GridCell(Coord2D(a.x, middle.y), Coord2D(middle.x, b.y), depth-1);
        south_east = new GridCell(middle, b, depth-1);
    } else {
        // Leaf cell
        nodes = std::unordered_set<MobilityNode*>();
    }
}

void GridCell::initializeCellNeighbors(bool isRoot) {
    // Initialize east_cell, south_cell

    if (depth == 0) {
        // In Leaf cell
        return;
    }

    if (not isRoot) {
        // Not in root cell
        if (east_cell != nullptr) {
            // Not on east bound
            north_east->east_cell = east_cell->north_west;
            south_east->east_cell = east_cell->south_west;
        }
        if (south_cell != nullptr) {
            // Not on south bound
            south_west->south_cell = south_cell->north_west;
            south_east->south_cell = south_cell->north_east;
        }
    }
    // Connect child cells
    north_west->east_cell = north_east;
    north_west->south_cell = south_west;
    north_east->south_cell = south_east;
    south_west->east_cell = south_east;

    // Recursivly initialize child cells
    north_west->initializeCellNeighbors(false);
    north_east->initializeCellNeighbors(false);
    south_west->initializeCellNeighbors(false);
    south_east->initializeCellNeighbors(false);
}

bool GridCell::isInside(Coord2D point) {
    return north_west_coord.x <= point.x
        && point.x <= south_east_coord.x
        && south_east_coord.y <= point.y
        && point.y <= north_west_coord.y;
}

GridCell* GridCell::findCell(Coord2D point) {
    if (depth == 0) {
        // No more child nodes
        ASSERT(isInside(point));
        return this;
    }

    // Find correct child cell
    if (north_west->isInside(point)) return north_west->findCell(point);
    if (north_east->isInside(point)) return north_east->findCell(point);
    if (south_east->isInside(point)) return south_east->findCell(point);
    if (south_west->isInside(point)) return south_west->findCell(point);

    ASSERT(false); // We only reach this if the given point is outside of this cell
    return nullptr;
}

void GridCell::insert(MobilityNode* node) {
    Coord2D pos = node->getMobilityModel()->getPosition();
    GridCell* cell = findCell(pos);
    ASSERT(cell != nullptr); // Assert that a cell was found

    EV_DEBUG << "Insert node " << pos.toString() << " into cell " << cell->north_west_coord.toString() << " " << cell->south_east_coord.toString() << endl;

    cell->nodes.emplace(node);
}

void GridCell::remove(MobilityNode* node, Coord2D oldPosition) {
    GridCell* cell = findCell(oldPosition);
    if (auto iter = cell->nodes.find(node); iter != cell->nodes.end()) {
        cell->nodes.erase(iter); // removes an element pointed to by iter
    }
}

std::string GridCell::toString() {
    std::string repr = "";
    for (int i = 0; i < 8 - depth; i++) {
        repr += "  ";
    }
    repr += std::to_string(depth) + " " + north_west_coord.toString()  + " " + south_east_coord.toString();
    repr += "\n";
    if (east_cell != nullptr && south_cell != nullptr) {
        repr += "East Cell: " + east_cell->printBounds() + " South Cell: " + south_cell->printBounds();
        repr += "\n";
    }
    if (depth > 0) {
        repr += north_west->toString() + "\n";
        repr += north_east->toString() + "\n";
        repr += south_west->toString() + "\n";
        repr += south_east->toString() + "\n";
    } else {
        for (int i = 0; i < 8 - depth; i++) {
            repr += "  ";
        }
        for (MobilityNode* n : nodes) {
            repr += std::to_string(n->getNodeId()) + " ";
        }
        repr += "\n";
    }

    return repr;
}

std::string GridCell::printBounds() {
    return north_west_coord.toString() + " " + south_east_coord.toString();
}

}
