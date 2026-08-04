package routingsim.routing.MYRIAD;

simple myriadgp extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadGP);

        // protocol — same hello/dead/rxmt knobs as MyriadLS so the shared
        // MyriadHelloProto can read them.
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
        double rxmtInterval @unit(s) = default(5s);
        // Jitter added to each hello: delay = helloInterval + uniform(0,
        // helloJitter). Decorrelates nodes so two-way / origination don't all
        // land on one tick. Keep helloInterval + helloJitter < deadInterval.
        // Default 0 (safe for the shared base); GP configs set it where the
        // synchronized-origination wave needs breaking up.
        double helloJitter @unit(s) = default(0s);

        // king re-originates a COLLECT every refreshInterval; members run a
        // watchdog of 2*refreshInterval + uniform(0, refreshJitter).
        double refreshInterval @unit(s) = default(30s);
        double refreshJitter @unit(s) = default(5s);

        // Best-path cost exponent a: a chain's size cost is Σ member_count^a.
        // a>1 is convex so one huge ring is penalised more than several small
        // rings of equal total size; a=1 is plain member sum. See ADR 0016.
        double ringCostExponent = default(2.0);

        // Inter-ring UPDATE batching: a forwarder becoming ring-king for a
        // transition into another ring holds the send for up to holdInterval,
        // coalescing every chain transitioning into the same (ring, neighbor)
        // this window into one MyriadGpUpdateBatch. Only inter-ring transitions
        // are held; intra-ring lap forwards are always immediate (else the
        // king's circular-ACK stalls and retransmits fire). 0s = off (exact
        // pre-batching behaviour: every transition sent immediately).
        double holdInterval @unit(s) = default(200ms);

        // Test hook: if non-empty, each node APPENDS its converged preferred
        // routes to this file in finish() (one line per (node, origin)). Used by
        // test_rib_batching.py to assert batching doesn't change the RIB. Empty
        // in production (no I/O).
        string ribDumpFile = default("");

        // Ring membership override, e.g. "A,B". Empty = use the built-in
        // per-id table. Set per node via **.NodeN.*.ringLabel or a topology
        // node property.
        string ringLabel = default("");

        // logging: DEBUG | INFO | WARN | ERROR
        string logLevel = default("DEBUG");
        // ANSI-color the log prefix by level (Cmdenv terminal only; off for log files / Qtenv)
        bool colorLog = default(false);
}
