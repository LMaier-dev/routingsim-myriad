package routingsim.routing.MYRIAD;

simple myriadls extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadLS);

        // protocol
        double helloInterval @unit(s) = default(4s);
        double deadInterval @unit(s) = default(9s);
        double rxmtInterval @unit(s) = default(2s);
        // Jitter added to each hello. Default 0: LS's DBD master/slave
        // negotiation reads hello ordering, so jitter would wedge it.
        double helloJitter @unit(s) = default(2s);
        // One-time stagger of the first hello at cold start; startup only (reboots
        // are spread by the failure scenario). A single offset, not per-hello.
        double joinJitter @unit(s) = default(0s);
        // Max number of LSA headers per DBD packet, or LsaRequests per LSR packet.
        int dbExchangeBatchSize = default(220);
        // OSPF MinLSInterval: at most one self-LSA origination per (origin,type)
        // per this window; bursts (e.g. many neighbors reaching FULL at once, or
        // a high-degree node) coalesce into one re-origination at the window edge.
        double minLsInterval @unit(s) = default(1s);

        // Ring membership, e.g. "A,B". Set per node via **.NodeN.*.ringLabel.
        string ringLabel = default("");

        // Optimality-oracle hook (see RingSpfOracle): non-empty oracleDumpFile makes
        // finish() append this node's Dijkstra least-cost-per-origin. Exponent must match PV's.
        int ringCostExponent = default(2);
        // Must match PV's; see MyriadPv.ned.
        int echoRingSize = default(20);
        string oracleDumpFile = default("");

        // Observational oracle snapshot period (see startSnapshot). >0 appends the
        // Dijkstra least-cost-per-origin to oracleDumpFile every interval with a
        // t=<simTime> prefix; 0s = off
        double snapshotInterval @unit(s) = default(1000ms);

        // See MyriadPv.ned.
        string snapshotFormat = default("full");

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("warn");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}