#include "BaseTypes.h"
#include "MobilityNode.h"
#include <vector>

namespace routingsim {

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
