
package routingsim.mobility;

simple MobilityBase extends Environment {
    parameters:
        @class(MobilityBase);
        double updateInterval @unit(s) = default(0.1s);
    gates:
        input inMobility @directIn;
}
