
package routingsim.mobility;

import routingsim.foundation.Node;

simple MobilityNode extends Node {
parameters:
    // for inheritance
    @class(MobilityNode);

    /// mobility model to use
    string mobilityModel = default("RandomWaypoint");
}
