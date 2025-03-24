#include "BaseTypes.h"
#include "MobilityNode.h"
#include <unordered_set>

namespace routingsim {

class GridCell {

public:
    // Recurcivly create GridCell
    GridCell(Coord2D a, Coord2D b, int depth);

    // Recursivly set east_cell,south_cell in top down order
    void initializeCellNeighbors(bool isRoot);

    // String representation of this cell for debugging
    std::string toString();

    // String represenation of the bounds of this cell for debugging
    std::string printBounds();

    // Returns the coordinate of the west bound of this cell
    Coord2D getNorthWestBound();

    // Returns the coordinate of the south east bound of this cell
    Coord2D getSouthEastBound();

    // Returns the cell to the east
    GridCell* getEastCell();

    // Returns the cell to the south
    GridCell* getSouthCell();

    // Checks wether the given point is inside this cell
    bool isInside(Coord2D point);

    // Inserts the given node into the correct child cell
    void insert(MobilityNode* node);

    // Removes the given node from this node
    void remove(MobilityNode* node, Coord2D oldPosition);

    // Returns the Leaf Cell with the given coordinates
    GridCell* findCell(Coord2D point);

    // Return the nodes of this cell (only works on leaf cells)
    std::unordered_set<MobilityNode*> getNodes();

private:
    // Upper left and lower right coordinates of this cell
    Coord2D north_west_coord, south_east_coord;

    // Depth of this cell (0 means leaf cell)
    int depth;

    // Child grids of this grid
    GridCell *north_west, *north_east, *south_east, *south_west;

    // Neighbor cells
    GridCell *east_cell, *south_cell;

    // Nodes inside this cell
    std::unordered_set<MobilityNode*> nodes;

};


class RegularGrid {

public:

    RegularGrid(double width,
                double height,
                int depth,
                bool useCache = false,
                bool sparseUpdates = false) {
        this->width = width;
        this->height = height;
        this->useCache = useCache;
        this->sparseUpdates = sparseUpdates;
        max_range = 0;
        next_update = -1; // Update on first call
        updates_to_do = 0;
        root_cell = new GridCell(Coord2D(0, height), Coord2D(width, 0), depth);
        root_cell->initializeCellNeighbors(true);
    };

    // Inserts the given MobilityNode into the correct grid cell
    void insert(MobilityNode* node);

    // Updates the node inside the grid when the nodes position got changed
    void update(MobilityNode* node, Coord2D oldPosition, double currentTime);

    // Returns nodes that may be in antenna range of the given node
    std::unordered_set<MobilityNode*> getNodesNear(MobilityNode* node);

private:
    // Bounds of the grid
    double width, height;

    // Maximum antenna range
    double max_range;

    // Wether or not to cache near nodes until some node switches cells
    bool useCache;
    // Wether or not to shortcut the grid cell node position update
    bool sparseUpdates;

    // Grid cell spanning the whole grid (contains up to `depth` levels of children cells)
    GridCell* root_cell;

    // Number of nodes inside the grid
    int number_of_nodes;

    // Number of updates that need to be done in the current update cycle
    int updates_to_do;

    // Time at which the near nodes cache should be updated
    double next_update;

    // Current guess when near nodes cache should be updated
    double next_update_guess;

    std::map<MobilityNode*, std::unordered_set<MobilityNode*>> near_nodes_cache;

    // Calculate the time until the cell cache needs to be erased
    double getNextCellChange(MobilityNode* node, GridCell* newCell);

};

}
