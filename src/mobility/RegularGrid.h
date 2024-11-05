#include "BaseTypes.h"
#include "MobilityNode.h"
#include <vector>

namespace routingsim {

class GridCell {

public:
    // Recurcivly create GridCell
    GridCell(Coord2D a, Coord2D b, int depth);

    // String representation of this cell for debugging
    std::string toString();

    // Checks wether the given point is inside this cell
    bool isInside(Coord2D point);

    // Inserts the given node into the correct child cell
    void insert(MobilityNode* node);

private:
    // Upper left and lower right coordinates of this cell
    Coord2D north_west_coord, south_east_coord;

    // Depth of this cell (0 means leaf cell)
    int depth;

    // Child grids of this grid
    GridCell *north_west, *north_east, *south_east, *south_west;

    // Nodes inside this cell
    std::vector<MobilityNode*> nodes;

};


class RegularGrid {

public:

    RegularGrid(double width, double height, int depth) {
        nodes = std::vector<MobilityNode*>();
        root_cell = new GridCell(Coord2D(0, height), Coord2D(width, 0), depth);
    };

    // Inserts the given MobilityNode into the correct grid cell
    void insert(MobilityNode* node);

    // Updates the node inside the grid when the nodes position got changed
    void update(MobilityNode* node, Coord2D oldPosition);

    // Returns nodes that may be in antenna range of the given node
    std::vector<MobilityNode*> getNodesNear(MobilityNode* node);

private:
    // TODO Remove when not needed anymore
    std::vector<MobilityNode*> nodes;

    // Grid cell spanning the whole grid (contains up to `depth` levels of children cells)
    GridCell* root_cell;

};

}
