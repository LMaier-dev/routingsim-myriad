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

        // Best-path cost exponent a (INTEGER, a>=1): a chain's size cost is
        // Σ member_count^a, computed in uint64 so equal-cost ties are exact
        // (enables ECMP co-best retention). a>1 is convex so one huge ring is
        // penalised more than several small rings of equal total size; a=1 is
        // plain member sum. See ADR 0016.
        int ringCostExponent = default(2);

        // ECMP fan-out cap: keep at most this many co-best (equal spof/cost/len)
        // chains per origin. On symmetric meshes the number of equal-cost paths
        // is combinatorial (a 10x10 grid has C(16,8)=12870 to some origins), so
        // an uncapped co-best set explodes both memory and inter-ring re-adverts.
        // When the set is full we keep the MOST RING-DIVERSE subset (greedy: drop
        // the member whose removal leaves the largest union of distinct rings).
        // Only the kept chains transition inter-ring. 1 = no multipath.
        int maxEcmpPaths = default(3);

        // Max serialized size (bytes) of one inter-ring UPDATE batch. At flush,
        // held transitions are packed into batches up to this size; the overflow
        // spills into additional batches that lap independently. Real path-vector
        // protocols cap PDU size the same way (BGP UPDATE 4096B classic; IS-IS
        // LSP ~1492B) — an unbounded batch can't fit one L2 frame. Default 1400
        // ≈ one Ethernet payload. Model size per entry = chain*8 + rr*6 + 16.
        int maxBatchBytes = default(1400);

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
