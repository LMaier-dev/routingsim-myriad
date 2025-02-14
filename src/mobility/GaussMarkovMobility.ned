package routingsim.mobility;

simple GaussMarkovMobility extends MobilityBase {
    parameters:
        @class(GaussMarkovMobility);
        // Gauss-Markov Mobility Degree of Memory
        double beta = default(1);
        // mu_|v|
        double speedMean;
        // sigma_|v|
        double speedVariance;
        // mu_theta
        double headingMean = default(0);
        // simga_theta
        double headingVariance;
}
