
simple AntennaBase {
    parameters:
        double minR @unit(m) = default(1m);
        double maxR @unit(m) = default(100m);
        double bandwidthMobilityMbps @unit(Mbps) = default(1000Mbps);
}

