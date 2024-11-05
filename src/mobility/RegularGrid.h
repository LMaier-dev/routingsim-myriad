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

private:
    // Upper left and lower right coordinates of this cell
    Coord2D south_west_coord, north_east_coord;

    // Depth of this cell (0 means leaf cell)
    int depth;

    // Child grids of this grid
    GridCell *south_west, *south_east, *north_west, *north_east;

};


class RegularGrid {

public:

    RegularGrid() {
        nodes = std::vector<MobilityNode*>();
    };

    // Inserts the given MobilityNode into the correct grid cell
    void insert(MobilityNode* node);

    // Updates the node inside the grid when the nodes position got changed
    void update(MobilityNode* node, Coord2D oldPosition);

    // Returns nodes that may be in antenna range of the given node
    std::vector<MobilityNode*> getNodesNear(MobilityNode* node);

private:
    std::vector<MobilityNode*> nodes;

};

}
