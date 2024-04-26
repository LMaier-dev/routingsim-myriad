
package routingsim.mobility;

import routingsim.foundation.Node;

simple MobilityNode extends Node {
parameters:
    ///@display("i=block/join");

    /// node id, -1 = choose own id
    int id = default(-1);

    /// protocols to the node uses
    /// string protocols;

    // minimum and maximum delay in milliseconds
    /// int nodeMinDelay @unit(us) = default(0us);
    /// int nodeMaxDelay @unit(us) = default(500us);

    /// mobility model to use
    string mobilityModel = default("RandomWaypoint");

///gates:
    ///input directIn @directIn;
    ///inout toProtocols[];
    ///inout toNeighbors[];
}