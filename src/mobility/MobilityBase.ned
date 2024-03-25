
simple MobilityBase {
    parameters:
        double updateInterval @unit(s) = default(0.1s);
    gates:
        input inMobility @directIn;
}
