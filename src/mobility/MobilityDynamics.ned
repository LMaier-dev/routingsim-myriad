
package routingsim.mobility;

simple MobilityDynamics {
    parameters:
        double updateInterval @unit(s) = default(0.1s);
        string topologyStatistics @enum(None,Light,Heavy) = default("Light");

    gates:
        output toManager;
}
