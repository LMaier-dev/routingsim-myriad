
package routingsim.mobility;

simple Environment {
    parameters:
        double borderX @unit(m) = default(1000m);
        double borderY @unit(m) = default(600m);
        double systemNoise @unit(dbm) = default(-172dbm);
}
