
package routingsim.mobility;

simple AntennaBase {
    parameters:
        //double minR @unit(m) = default(1m);
        //double maxR @unit(m) = default(100m);
        //double bandwidthMobilityMbps @unit(Mbps) = default(1000Mbps);
        double minP @unit(dbm) = default(10dbm);
        double maxP @unit(dbm) = default(12dbm);
        double minB @unit(MHz) = default(10MHz);
        double maxB @unit(MHz) = default(12MHz);
}

