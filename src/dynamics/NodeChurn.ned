package routingsim.dynamics;

import routingsim.foundation.NetworkDynamics;

/// Steady low-rate node churn: `numNodes` random nodes go down every `period`
/// and come back after downMin + U(0, downJitter). Unlike EscalatingFailure the
/// rate never grows, so the network is never knocked over — it measures what a
/// protocol costs to keep a large topology converged while nodes keep leaving
/// and rejoining, rather than how it recovers from a wave.
simple NodeChurn extends NetworkDynamics {
parameters:
    @class(NodeChurn);

    /// nodes failing per period, and how often
    int numNodes = default(1);
    double period @unit(s) = default(1s);

    /// first failure
    double warmup @unit(s) = default(10s);

    /// downtime per node: downMin + U(0, downJitter)
    double downMin @unit(s) = default(2s);
    double downJitter @unit(s) = default(8s);

    /// degree range of nodes eligible to fail (maxDegree 0 = no upper bound)
    int minDegree = default(0);
    int maxDegree = default(0);
}
