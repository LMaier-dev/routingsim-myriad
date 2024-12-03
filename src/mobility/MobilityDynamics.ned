
package routingsim.mobility;

simple MobilityDynamics {
    parameters:
        double updateInterval @unit(s) = default(0.1s);
        string topologyStatistics @enum(None,Light,Heavy) = default("Light");
        double borderX @unit(m) = default(1000m);
        double borderY @unit(m) = default(600m);
        int gridDepth = default(1);
        bool useGrid;
        bool useNearNodesCache;

    gates:
        output toManager;
}
