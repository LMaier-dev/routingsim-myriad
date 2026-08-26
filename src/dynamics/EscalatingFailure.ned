package routingsim.dynamics;

import routingsim.foundation.NetworkDynamics;

/// Node failures in waves of growing size: wave i fails fractions[i] of the
/// nodes at warmup + i*waveSpacing. One run per failure-size sweep.
simple EscalatingFailure extends NetworkDynamics {
parameters:
    @class(EscalatingFailure);

    /// comma-separated failure fractions, one per wave
    string fractions = default("0.05,0.1,0.2,0.3");

    /// first wave, and the time between waves
    double warmup @unit(s) = default(30s);
    double waveSpacing @unit(s) = default(30s);

    /// downtime per node: downMin + U(0, downJitter)
    double downMin @unit(s) = default(2s);
    double downJitter @unit(s) = default(8s);

    /// degree range of nodes eligible to fail (maxDegree 0 = no upper bound)
    int minDegree = default(0);
    int maxDegree = default(0);
}
