package routingsim.mobility;

simple RandomWaypointMobility extends MobilityBase {
    parameters:
        volatile double minSpeed @unit(mps) = default(1mps);
        volatile double maxSpeed @unit(mps) = default(10mps);
        volatile double waitTime @unit(s) = default(0s);
}
