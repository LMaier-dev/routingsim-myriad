package routingsim.routing.MYRIAD;

simple myriadgp extends routingsim.foundation.Protocol
{
    parameters:
        @class(MyriadGP);

        // Shared with MyriadLS via MyriadHelloProto. Keep helloInterval +
        // helloJitter < deadInterval. GP configs raise helloJitter to break up
        // the synchronized-origination wave.
        double helloInterval @unit(s) = default(10s);
        double deadInterval @unit(s) = default(40s);
        double rxmtInterval @unit(s) = default(5s);
        double helloJitter @unit(s) = default(0s);

        // King re-originates a COLLECT every refreshInterval; members watchdog at
        // 2*refreshInterval + uniform(0, refreshJitter).
        double refreshInterval @unit(s) = default(30s);
        double refreshJitter @unit(s) = default(5s);

        // Path-cost exponent for summing up member_count^a; see MyriadGp.h.
        // a>1 is convex (penalises one huge ring over several small ones), a=1
        // is a plain member sum.
        int ringCostExponent = default(2);

        // Rank on SPOF count first (default) or skip it and rank purely by cost.
        // false makes the RIB least-cost, matching the Dijkstra algo
        bool rankUseSpof = default(true);

        // ECMP fan-out cap; excess co-best chains dropped keeping the most
        // ring-diverse subset (see capEcmp). 1 = no multipath. Needed because
        // equal-cost paths are combinatorial on symmetric meshes.
        int maxEcmpPaths = default(3);

        // Max serialized bytes of one inter-ring UPDATE batch; the flush splits
        // the held set across frame-sized batches (real path-vector protocols
        // cap PDU size too). 1400 ≈ one Ethernet payload.
        int maxBatchBytes = default(1400);

        // Inter-ring UPDATE batching window (see flushPendingTransitions); only
        // inter-ring transitions are held, never intra-ring laps. 0s = off.
        double holdInterval @unit(s) = default(200ms);

        // Test hook (see finish / test_rib_batching.py): if non-empty, each node
        // appends its converged preferred routes here.
        string ribDumpFile = default("");

        // Observational RIB snapshot period (see startSnapshot). >0 appends the
        // preferred RIB to ribDumpFile every interval with a t=<simTime> prefix;
        // 0s = off (finish() dump only). Read-only, does not perturb the protocol.
        double snapshotInterval @unit(s) = default(0s);

        // Periodic snapshot format: "full" (one line per route, needed for
        // cost-drift) or "stats" (one aggregate line per node: route count, cost
        // sum + max)
        string snapshotFormat = default("full");

        // Warmup: run COLLECTs (establishing memberCounts) for this duration
        // before originating any UPDATEs. Eliminates the first-round convergence
        // artifact caused by inter-ring UPDATEs arriving before the local COLLECT
        // has set memberCount. Convergence measurements subtract this offset.
        // 0s = off (default; normal protocol behaviour).
        double collectWarmupDuration @unit(s) = default(0s);

        // Interval at which COLLECTs are re-triggered during warmup. Should be
        // long enough for one COLLECT to complete (~ms in practice, 1s is safe).
        double collectWarmupInterval @unit(s) = default(1s);

        // Ring membership override, e.g. "1000000,999999". Empty = built-in
        // per-id table. Set via **.NodeN.*.ringLabel or a node property.
        string ringLabel = default("");

        string logLevel = default("DEBUG");  // DEBUG | INFO | WARN | ERROR
        // ANSI-color the log prefix by level (Cmdenv terminal only).
        bool colorLog = default(false);
}
