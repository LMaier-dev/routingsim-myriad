#ifndef __ROUTINGSIM_MYRIADGP_H_
#define __ROUTINGSIM_MYRIADGP_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadGpPkt_m.h"
#include <boost/container/flat_map.hpp>
#include <unordered_map>
#include <map>
#include <vector>
#include <functional>

namespace routingsim {

    struct GpNeighbor : HelloNeighbor {
    };

    // In-flight reliable ring send. Retransmits alternate primary/fallback so
    // one directional send covers both ring directions across retries; equal
    // primary/fallback degenerates to same-neighbor retry (probe mode).
    struct RingTx {
        cPacket* pkt = nullptr;   // template; dup()'d on each (re)send
        nodeid_t primary = 0;
        nodeid_t fallback = 0;
        simtime_t interval;
        TimerMsg* timer = nullptr;
        bool onFallback = false;  // which neighbor the next retransmit targets
    };

    // When packet is superseeded by a newer SN, also cancel RTX and flood this one.
    struct RingInflight {
        RingTx* tx = nullptr;
        // seq_no of the packet this inflight is circulating; matched on own-return.
        uint32_t seq_no = 0;
        // Parallel budget timer of (retryLimit+0.5)*rxmtInterval; on fire while
        // tx still set → ring declared broken.
        TimerMsg* budgetTimer = nullptr;
    };

    // One learned path to an origin ring. chain is prepend-order:
    // chain[0] = latest ring visited, chain.back() = origin.
    struct RibEntry {
        std::vector<uint32_t> chain;
        std::vector<uint32_t> chain_sizes; // raw member count per ring, parallel to chain
        uint32_t seq_no = 0;
        uint32_t spof_count = 0;
    };

    // Collect phase fills this for adjacent rings
    // false: single router on this ring also in `ring`
    // true: at least two routers on this ring also in `ring` (dont care for more than two)
    struct AdjacentRingState {
        uint32_t ring;
        bool isRedundant;
    };

    struct RingState {
        std::vector<MyriadGpRr> rr; // (subnet, owner) pairs; refreshed on curSeq advance
        uint32_t curSeq = 0; // highest seq_no seen for this origin ring
        // My own sequence for this ring, shared by COLLECT and origin-UPDATE.
        uint32_t originSeq = 0;
        // My collect for this ring (at most one at a time).
        RingInflight collectInflight;
        // Reliable UPDATE traversals I currently own in this ring, keyed by the
        // packet's chain: empty chain = my origin-UPDATE; non-empty = a transit
        // relay of another origin's UPDATE. Several can coexist (different chains).
        std::map<std::vector<uint32_t>, RingInflight> updateInflights;
        // Whole-batch inter-ring transition laps I own in this ring (hold>0
        // path), keyed by the batch-lap seq I stamped. Distinct from
        // updateInflights: a batch mixes many chains but closes ONE circular-ACK.
        std::map<uint32_t /*batchLapSeq*/, RingInflight> batchInflights;
        // Monotonic per-ring counter for batch-lap seqs I originate here.
        uint32_t batchLapSeq = 0;
        std::vector<RibEntry> ribIn; // all currently-valid chains for this origin
        size_t preferred = SIZE_MAX; // index into ribIn; SIZE_MAX = none

        // derived from recent Collect lap; used for spof_count on inter-ring forward
        std::vector<AdjacentRingState> adjacentRings;
        // This ring's member count (its size). Set from the king's own COLLECT
        // return, or from the origin-UPDATE trailer for non-king members
        uint32_t memberCount = 0;

        // Persistent kingship + periodic refresh (king) / liveness watchdog (member).
        bool isKing = false;
        TimerMsg* refreshTimer = nullptr;   // king only: periodic re-collect
        TimerMsg* watchdogTimer = nullptr;  // member only: coup if king goes silent
        // High-water mark of ring-neighbors ever seen. ==1 means a genuine
        // 2-node ring (echo to close the lap); dropping from 2 to 1 is
        // degradation (let the lap fail so DEGRADE fires).
        size_t maxRingNeighbors = 0;
    };

    // A materialized routing-table entry.
    struct RouteEntry {
        uint32_t origin_ring = 0;
        std::vector<uint32_t> chain; // prepend-order, same as RibEntry
        nodeid_t owner = 0;
    };

    // One inter-ring UPDATE transition held for batching (see holdInterval).
    // Captures everything buildTransitionUpdate needs to (re)build the single-
    // chain fwd, plus the send targets. origin = fwdChain.back().
    struct PendingTransition {
        uint32_t R2 = 0;
        nodeid_t primary = 0;
        nodeid_t fallback = 0;
        uint32_t seq = 0;
        uint32_t spof = 0;
        std::vector<uint32_t> fwdChain;
        std::vector<uint32_t> fwdSizes;
        std::vector<MyriadGpRr> rr;
    };

    class MyriadGP : public MyriadHelloProto<GpNeighbor>
    {
        // Distinct from MYRIADLS_PACKET_TYPE so both protocols can coexist.
        static constexpr int MYRIADGP_PACKET_TYPE = 56;

        protected:
            void onInitialize() override;
            void onMessage(NodePkt_p pkt) override;
            void reset() override;
            void finish() override;

            void onTwoWayReached(GpNeighbor& nbr) override;
            void onNeighborDead(GpNeighbor& nbr, bool wasFull) override;

            void handleCollect(MyriadGpCollect* collect, nodeid_t from);
            void handleUpdate(MyriadGpUpdate* update, nodeid_t from);
            void handleUpdateBatch(MyriadGpUpdateBatch* batch, nodeid_t from);
            void handleDegrade(MyriadGpDegrade* degrade, nodeid_t from);

            //  Originate a COLLECT for ring R towards `to`.
            void originateCollect(uint32_t ring, nodeid_t to);
            // Originate a fresh COLLECT for a ring I own (picks a ring-neighbor).
            void reoriginateCollect(uint32_t ring);
            // Persistent-king refresh timer and member liveness watchdog.
            void startRefresh(uint32_t ring);
            void startWatchdog(uint32_t ring);
            void cancelKingship(uint32_t ring);
            // Insert my (subnet,myId) pairs + update transitions + forward to
            // the other ring-neighbor for R. `sender` is the immediate hop we
            // received from (NOT the king), used to pick the other neighbor.
            void insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t ring, nodeid_t sender);
            // The other ring-neighbor for R (the one we did NOT receive from);
            // returns undefined_node if none exists.
            nodeid_t otherRingNeighbor(uint32_t ring, nodeid_t sender);
            // Both TWOWAY+ ring-neighbors for R (0, 1, or 2 entries).
            std::vector<nodeid_t> ringNeighbors(uint32_t ring);
            // Originate an UPDATE for a ring I just finished collecting.
            void originateUpdate(uint32_t ring);
            // Forward a single origin/relay UPDATE within its current_ring
            // (unconditional, closes the king's lap).
            void forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender);
            // Forward a whole batch within current_ring (unconditional). The batch
            // never unpacks intra-ring: it laps as one packet to close ONE ACK.
            void forwardBatchInRing(MyriadGpUpdateBatch* pkt, uint32_t ring, nodeid_t sender);
            // Process ONE learned chain at this node: RIB insert/select, and if it
            // wins + is loop-free, transition it inter-ring (appending R2s it fans
            // into to `handled`). Shared by handleUpdate (its single chain) and
            // handleUpdateBatch (each entry). Returns nothing; `handled` accrues
            // the rings this chain was flooded into so the caller can mark the
            // lapping packet. curRing = the ring this chain is currently lapping.
            void processChain(uint32_t curRing, uint32_t seq, uint32_t spof,
                              const std::vector<uint32_t>& chain,
                              const std::vector<uint32_t>& chainSizes,
                              const std::vector<MyriadGpRr>& rr,
                              const std::vector<MyriadGpRr>* originRrTrailer,
                              std::vector<uint32_t>& handled);
            // Transition one chain into each of my other rings not in `chain` and
            // not already in `handled`. hold=0 sends a single-chain UPDATE now
            // (old path, exact pre-batch behaviour); hold>0 queues it for the
            // next batch flush. Appends each R2 it fans into to `handled`.
            void forwardChainInterRing(uint32_t curRing, uint32_t seq, uint32_t spof,
                              const std::vector<uint32_t>& chain,
                              const std::vector<uint32_t>& chainSizes,
                              const std::vector<MyriadGpRr>& rr,
                              std::vector<uint32_t>& handled);
            // Build a single-chain inter-ring transition UPDATE (current_ring=R2,
            // I own it) from raw fields — the hold=0 immediate path.
            MyriadGpUpdate* buildTransitionUpdate(uint32_t R2, nodeid_t src, uint32_t seq,
                                  uint32_t spof, const std::vector<uint32_t>& fwdChain,
                                  const std::vector<uint32_t>& fwdSizes,
                                  const std::vector<MyriadGpRr>& rr);
            // Flush all held inter-ring transitions (hold>0): re-validate each,
            // then emit one MyriadGpUpdateBatch per (R2, primary) bucket that laps
            // R2 whole under a single batch-lap circular-ACK.
            void flushPendingTransitions();
            // Cancel a collect inflight + its budget timer for `ring`.
            void cancelCollect(uint32_t ring);
            // Budget-timer expiry for a collect on `ring`: ring declared broken.
            void onCollectBudgetExpired(uint32_t ring);
            // Budget-timer expiry for an UPDATE traversal (ring, chain).
            void onUpdateBudgetExpired(uint32_t ring, std::vector<uint32_t> chain);
            // Budget-timer expiry for a whole-batch lap (ring, batchLapSeq).
            void onBatchBudgetExpired(uint32_t ring, uint32_t batchLapSeq);
            // Retransmit budget (retransmissions after the initial send).
            static constexpr int COLLECT_RETRY_LIMIT = 3;
            // Placeholder ring size before a ring's origin-UPDATE trailer arrives
            static constexpr uint32_t UNKNOWN_RING_SIZE = 1u << 20;

            // Reliable ring send: first send to `primary`, retransmits alternate
            // to `fallback` (equal args = same-neighbor probe). Sets fl.tx and
            // fl.budgetTimer; onBudgetExpired fires after retryLimit misses.
            // Takes ownership of `pkt`.
            void sendRingReliable(RingInflight& fl, nodeid_t primary, nodeid_t fallback,
                                  cPacket* pkt, simtime_t interval, int retryLimit,
                                  std::function<void()> onBudgetExpired);
            void cancelRingReliable(RingInflight& fl);
            void ringRetxFire(RingTx* h);
            // (subnet, myId) pairs derived once at onInitialize; reused as
            // both the local-subnet source at origination and on insert.
            std::vector<uint32_t> mySubnets;
            // King re-collect period, and member watchdog jitter bound.
            simtime_t refreshInterval;
            simtime_t refreshJitter;
            // Best-path cost exponent a: path cost = sum of (member_count^a) over the chain
            double ringCostExponent;
            // Inter-ring transition batching window; 0 = off (send immediately).
            simtime_t holdInterval;
            // Held inter-ring transitions bucketed by (target ring R2, next-hop
            // primary), flushed together when the hold timer fires.
            std::map<std::pair<uint32_t, nodeid_t>, std::vector<PendingTransition>> pendingTransitions;
            TimerMsg* holdTimer = nullptr;   // single per-node; armed on first held chain
            // Batch-size instrumentation: total entries flushed and largest
            // single batch, recorded as scalars in finish() (avg = entries/count
            // of update_batch sends). ponytail: plain counters, not a full
            // cOutVector histogram — the sweep only needs mean + max.
            unsigned long batchEntriesTotal = 0;
            unsigned long batchMaxEntries = 0;
            // Distinct batches created (flushed), the denominator for mean batch
            // size — NOT sent_update_batch, which also counts every lap-forward.
            unsigned long batchesCreated = 0;
            double pathCost(const RibEntry& e) const;
            // Recompute rings[origin].preferred
            void selectPreferred(RingState& rs);

        private:
            std::unordered_map<uint32_t /*ring*/, RingState> rings;
            // Degrade state per culprit ring: {seq_no, beacon_no}. Held here (not
            // per-chain) so RIB-In purges don't lose it; chains re-mark from this.
            std::unordered_map<uint32_t /*culprit ring*/,
                               std::pair<uint32_t /*seq_no*/, uint32_t /*beacon_no*/>> degrades;
            // Materialized routing table: subnet → preferred route.
            std::unordered_map<uint32_t /*subnet*/, RouteEntry> routes;
            // Reverse lookup: subnet → ring(s) that advertise it
            std::unordered_map<uint32_t /*subnet*/, std::vector<uint32_t> /*ring_ids*/> subnetToRing;
            std::vector<uint32_t /*subnet*/> keys; // only for simulation later to pick destinations at random
    };

} // namespace routingsim

#endif
