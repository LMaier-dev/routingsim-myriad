package routingsim.routing.MYRIAD;

simple myriadls extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadLS);

        // protocol
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
        double rxmtInterval @unit(s) = default(5s);
        // Jitter added to each hello. Default 0: LS's DBD master/slave
        // negotiation reads hello ordering, so jitter would wedge it.
        double helloJitter @unit(s) = default(0s);
        // Max number of LSA headers per DBD packet, or LsaRequests per LSR packet.
        int dbExchangeBatchSize = default(100);

        // Ring membership, e.g. "A,B". Set per node via **.NodeN.*.ringLabel.
        string ringLabel = default("");

        // Optimality-oracle hook (see RingSpfOracle): non-empty oracleDumpFile makes
        // finish() append this node's Dijkstra least-cost-per-origin. Exponent must match GP's.
        int ringCostExponent = default(2);
        string oracleDumpFile = default("");

        // Observational oracle snapshot period (see startSnapshot). >0 appends the
        // Dijkstra least-cost-per-origin to oracleDumpFile every interval with a
        // t=<simTime> prefix; 0s = off
        double snapshotInterval @unit(s) = default(0s);

        // See MyriadGp.ned.
        string snapshotFormat = default("full");

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("DEBUG");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}