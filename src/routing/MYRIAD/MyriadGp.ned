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
        // One-time stagger of the first hello at cold start (uniform(0, joinJitter)),
        // so ring formation isn't a synchronized herd. Reboots are already spread
        // by the failure scenario, so this applies to initial startup only.
        double joinJitter @unit(s) = default(0s);

        // King re-originates a COLLECT every refreshInterval; a member watchdogs
        // at refreshInterval + refreshJitter + watchdogMargin
        double refreshInterval @unit(s) = default(30s);
        double refreshJitter @unit(s) = default(refreshInterval * 0.1);
        double watchdogMargin @unit(s) = default(2s);

        // Spread the first propagating UPDATE. Inter-ring forwarding is gated
        // until collectWarmupDuration.
        double updateJitter @unit(s) = default(10s);

        // Path-cost exponent for summing up member_count^a; see MyriadGp.h.
        // a>1 is convex (penalises one huge ring over several small ones), a=1
        // is a plain member sum.
        int ringCostExponent = default(2);

        // Size a 2-member ring is priced at instead of its real one. Such a ring
        // is a single link carrying both of its spofs, so it is a last resort
        // however cheap its size looks; 2 disables the penalty.
        int echoRingSize = default(64);

        // Rank on SPOF count first (default) or skip it and rank purely by cost.
        // false makes the RIB least-cost, matching the Dijkstra algo
        bool rankUseSpof = default(true);

        // Held transitions that ADD a spof flush at holdInterval * this, so the
        // spof-free wave settles the RIB first and the spof-incurring one — which
        // rankUseSpof would outrank anyway — dies on arrival instead of
        // propagating a full wave. 1 = off.
        double spofHoldMultiplier = default(5.0);

        // On a broken ring, flood a DEGRADE so every node demotes chains that
        // transit it below any non-degraded alternative until it recovers. Off =
        // rely on refresh/purge alone (broken paths linger until reconvergence).
        bool degradeEnabled = default(true);

        // Don't transition chains into a ring with an open degrade: its laps
        // aren't closing, so the send only burns the retransmit budget.
        bool skipDegradedTransitions = default(true);

        // Eval only: Disable RR
        bool carryRr = default(true);

        // On COLLECT own-return, if a recovered (stateless) member is on the
        // ring, the king replays its current front-of-ring chains as seq=0
        // UPDATEs so the recovered node repopulates without waiting for refresh.
        bool recoveryReplayEnabled = default(true);

        // false = the king's routine refresh (already king, nothing new) only
        // re-verifies ring state and resets watchdogs, without re-flooding an
        // UPDATE.
        bool refreshFloodsUpdate = default(true);

        // ECMP fan-out cap; excess co-best chains dropped keeping the most
        // ring-diverse subset (see capEcmp). 1 = no multipath. Needed because
        // equal-cost paths are combinatorial on symmetric meshes.
        int maxEcmpPaths = default(3);

        // Relative cost band counting two chains as co-best (0.2 = 20%). 0 =
        // exact equality, which on an irregular topology almost never holds,
        // leaving maxEcmpPaths inert. Must stay 0 for the Dijkstra oracle gate.
        double ecmpCostMargin = default(0.2);

        // Which co-best chain the ECMP cap evicts: true = the one leaving the
        // largest ring-union behind (a failover set stays ring-diverse), false =
        // the costliest, so the set is the K cheapest in-band chains. Only the
        // eviction order; acceptance is always the margin, and the cheapest
        // chain is preferred either way.
        bool ecmpDiverseEviction = default(true);

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
        // On by default: without it, round-1 UPDATEs carry unsettled counts and
        // corrections diffuse only ~1 ring-hop per refresh cycle. 0s = off.
        double collectWarmupDuration @unit(s) = default(10s);

        // Interval at which COLLECTs are re-triggered during warmup. 0s (the
        // default) derives it as 0.3 * collectWarmupDuration, i.e. ~3 COLLECTs
        // per ring across the window. On ring-dense topologies a shorter fixed
        // interval floods the network faster than laps can close (each expiring
        // into a DEGRADE), so keep this tied to the warmup length; raise the
        // warmup itself for large topologies rather than shortening this.
        double collectWarmupInterval @unit(s) = default(0s);

        // Ring membership override, e.g. "1000000,999999". Empty = built-in
        // per-id table. Set via **.NodeN.*.ringLabel or a node property.
        string ringLabel = default("");

        string logLevel = default("DEBUG");  // DEBUG | INFO | WARN | ERROR
        // ANSI-color the log prefix by level (Cmdenv terminal only).
        bool colorLog = default(false);
}
