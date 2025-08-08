package routingsim.mobility;

simple GaussMarkovMobility extends MobilityBase {
    parameters:
        @class(GaussMarkovMobility);
        // Gauss-Markov Mobility Degree of Memory
        double beta = default(1);
        // mu_|v|
        double speedMean @unit(mps);
        // sigma_|v|
        double speedStandardDeviation @unit(mps);
        // mu_theta
        double headingMean = default(0);
        // simga_theta
        double headingStandardDeviation;
        // border margins
        double borderMarginX @unit(m);
        double borderMarginY @unit(m);
}
