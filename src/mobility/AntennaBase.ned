
package routingsim.mobility;

simple AntennaBase {
    parameters:
        double minP @unit(dbm) = default(10dbm);
        double maxP @unit(dbm) = default(12dbm);
        double minB @unit(MHz) = default(10MHz);
        double maxB @unit(MHz) = default(12MHz);
        double systemNoise @unit(dbm) = default(-172dbm);
}

