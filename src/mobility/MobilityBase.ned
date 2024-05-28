
package routingsim.mobility;

simple MobilityBase extends Environment {
    parameters:
        @class(MobilityBase);
        double updateInterval @unit(s) = default(0.1s);
        int mobilityRNG = default(0);

    gates:
        input inMobility @directIn;
}
