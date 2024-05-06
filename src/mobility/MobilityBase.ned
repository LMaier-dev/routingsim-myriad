
package routingsim.mobility;

simple MobilityBase extends Environment {
    parameters:
        @signal[mobilityStateChanged](type=bool);
        @class(MobilityBase);
        double updateInterval @unit(s) = default(0.1s);
        int mobilityRNG = default(0);

    gates:
        input inMobility @directIn;
}
