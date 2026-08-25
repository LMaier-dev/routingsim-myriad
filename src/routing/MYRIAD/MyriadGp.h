#ifndef __ROUTINGSIM_MYRIADGP_H_
#define __ROUTINGSIM_MYRIADGP_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadGpPkt_m.h"
#include <boost/container/flat_map.hpp>
#include <unordered_map>
#include <map>
#include <vector>
#include <functional>
#include <iosfwd>
#include <memory>
#include <string>

namespace routingsim {

    struct GpNeighbor : HelloNeighbor {
    };

    // Reliable ring send. Retransmits alternate primary/fallback so one
    // directional send covers both ring directions; equal primary/fallback is
    // same-neighbor probing.
    struct RingTx {
        cPacket* pkt = nullptr;   // template; dup()'d on each (re)send
        nodeid_t primary = 0;
        nodeid_t fallback = 0;
        simtime_t interval;
        TimerMsg* timer = nullptr;
        bool onFallback = false;
    };

    struct RingInflight {
        RingTx* tx = nullptr;
        uint32_t seq_no = 0;      // matched when the packet laps back to us
        // Fires (retryLimit+0.5) intervals in; if tx is still set, ring is broken.
        TimerMsg* budgetTimer = nullptr;
    };

    // A learned path to an origin ring. chain is prepend-order:
    // chain[0] = latest ring, chain.back() = origin.
    struct RibEntry {
        std::vector<uint32_t> chain;
        uint64_t chain_cost = 0;           // sum of memberCount^ringCostExponent over chain
        uint32_t seq_no = 0;
        uint32_t spof_count = 0;
    };

    // isRedundant: false = one router bridges to this ring (a SPOF),
    // true = two or more do. Filled during the Collect lap.
    struct AdjacentRingState {
        uint32_t ring;
        bool isRedundant;
    };

    // State a router keeps ONLY for rings it is itself a member of: king/collect
    // bookkeeping and the in-flight ring laps.
    struct RingMembership {
        uint32_t originSeq = 1;     // my sequence for this ring (COLLECT + origin-UPDATE)
        uint32_t memberCount = 0;   // ring size; set when a COLLECT lap closes
        RingInflight collectInflight;
        // Reliable UPDATE traversals I own here, keyed by chain (empty = my
        // origin-UPDATE, non-empty = a relay); several coexist on distinct chains.
        std::map<std::vector<uint32_t>, RingInflight> updateInflights;
        // Batch laps I own here (hold>0). Unlike updateInflights, one batch mixes
        // many chains but closes a single circular-ACK, so it's keyed by lap seq.
        std::map<uint32_t /*batchLapSeq*/, RingInflight> batchInflights;
        uint32_t batchLapSeq = 0;   // monotonic counter for batch laps I originate
        std::vector<AdjacentRingState> adjacentRings; // feeds spof_count on forward
        bool isKing = false;
        TimerMsg* refreshTimer = nullptr;   // king: periodic re-collect
        TimerMsg* watchdogTimer = nullptr;  // member: coup if the king goes silent
        // High-water mark of ring-neighbors. 1 = genuine 2-node ring; a drop from
        // 2 to 1 is degradation, so we let the lap fail and DEGRADE fires.
        size_t maxRingNeighbors = 0;
    };

    struct RingState {
        // Origin-scoped: kept for EVERY ring this router knows a route to.
        std::vector<MyriadGpRr> rr; // (subnet, owner) pairs; refreshed when curSeq advances
        uint32_t curSeq = 0;        // highest seq_no seen for this origin ring
        // Learned chains for this origin. Usually one generation (all co-best).
        // Make-before-break may also keep PREVIOUS-generation chains (seq_no ==
        // curSeq-1) that beat the current one, so the data plane keeps a good
        // redundant route while reconverging.
        // At most two adjacent generations coexist.
        std::vector<RibEntry> ribIn;
        size_t preferred = SIZE_MAX; // index into ribIn; SIZE_MAX = none
        // Member-scoped state; present only if this router is on the ring.
        std::unique_ptr<RingMembership> member;
    };

    struct RouteEntry {
        uint32_t origin_ring = 0;
        std::vector<uint32_t> chain; // prepend-order, like RibEntry
        nodeid_t owner = 0;
    };

    // An inter-ring transition held for batching (see holdInterval); origin is
    // fwdChain.back().
    struct PendingTransition {
        uint32_t R2 = 0;
        nodeid_t primary = 0;
        nodeid_t fallback = 0;
        uint32_t seq = 0;
        uint32_t spof = 0;
        std::vector<uint32_t> fwdChain;
        uint64_t fwdCost = 0;
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
            // A recovered member needs its RIB back: replay every current-gen chain
            void replayInto(uint32_t ring);
            // COLLECT election key: live (memberCount>0) beats crashed, ties by id.
            bool collectWins(bool incomingCrashed, nodeid_t incomingId, uint32_t ring) const;
            // Null-safe member count: 0 if I hold no membership for `ring`
            uint32_t memberCountOf(uint32_t ring) const;
            // A broken ring's collect lap expired: record + flood a DEGRADE for it
            // (re-beaconed on each retry). floodDegrade skips the hop we got it from.
            void originateDegrade(uint32_t ring);
            void floodDegrade(uint32_t ring, uint32_t seq, uint32_t beacon, nodeid_t skip);
            // A chain is degraded iff any ring on it has an active degrade.
            bool chainDegraded(const std::vector<uint32_t>& chain) const;
            void reselectAll();
            // A newer generation of `ring` proves it healed → drop its degrade.
            void clearDegradeIfRecovered(uint32_t ring);

            void originateCollect(uint32_t ring, nodeid_t to, bool beacon = false);
            // Fresh COLLECT for a ring I own (picks a ring-neighbor).
            void reoriginateCollect(uint32_t ring);
            void startRefresh(uint32_t ring);   // king: periodic re-collect
            void startWatchdog(uint32_t ring);  // member: coup if the king goes silent
            // Observational periodic RIB snapshot (snapshotInterval)
            void startSnapshot();
            void dumpRibTo(std::ostream& f, const std::string& tprefix, bool stats);
            void startWarmupCollects();  // periodic COLLECT re-runs during warmup
            void cancelKingship(uint32_t ring);
            // Insert my pairs + transitions, forward to the other ring-neighbor.
            // `sender` is the hop we received from, used to pick the other one.
            void insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t ring, nodeid_t sender);
            // The ring-neighbor we did NOT receive from; undefined_node if none.
            nodeid_t otherRingNeighbor(uint32_t ring, nodeid_t sender);
            // Both TWOWAY+ ring-neighbors for R (0, 1, or 2).
            std::vector<nodeid_t> ringNeighbors(uint32_t ring);
            // True iff R is a genuine 2-node ring, from ring-ids neighbors advertise in hello
            bool isTwoNodeRing(uint32_t ring);
            void originateUpdate(uint32_t ring);
            void forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender);
            // Forward a whole batch intra-ring; it never unpacks, lapping as one
            // packet to close a single ACK.
            void forwardBatchInRing(MyriadGpUpdateBatch* pkt, uint32_t ring, nodeid_t sender);
            // RIB insert/select for one learned chain, then transition it
            // inter-ring if it wins and is loop-free. Shared by handleUpdate and
            // handleUpdateBatch (per entry). `handled` accrues the rings this
            // chain fanned into, so the caller can mark the lapping packet.
            void processChain(uint32_t curRing, uint32_t seq, uint32_t spof,
                              const std::vector<uint32_t>& chain,
                              uint64_t chainCost,
                              const std::vector<MyriadGpRr>& rr,
                              const std::vector<MyriadGpRr>* originRrTrailer,
                              std::vector<uint32_t>& handled);
            // Transition one chain into each of my other rings not in `chain` or
            // `handled`. hold=0 sends now; hold>0 queues for the next flush.
            void forwardChainInterRing(uint32_t curRing, uint32_t seq, uint32_t spof,
                              const std::vector<uint32_t>& chain,
                              uint64_t chainCost,
                              const std::vector<MyriadGpRr>& rr,
                              std::vector<uint32_t>& handled);
            // Build a single-chain transition UPDATE (the hold=0 immediate path).
            MyriadGpUpdate* buildTransitionUpdate(uint32_t R2, nodeid_t src, uint32_t seq,
                                  uint32_t spof, const std::vector<uint32_t>& fwdChain,
                                  uint64_t fwdCost,
                                  const std::vector<MyriadGpRr>& rr);
            // Emit the held transitions as one batch per (R2, primary) bucket,
            // each lapping R2 under a single circular-ACK.
            void flushPendingTransitions();
            void cancelCollect(uint32_t ring);
            void onCollectBudgetExpired(uint32_t ring);
            void onUpdateBudgetExpired(uint32_t ring, std::vector<uint32_t> chain);
            void onBatchBudgetExpired(uint32_t ring, uint32_t batchLapSeq);
            // Retransmits after the initial send.
            static constexpr int COLLECT_RETRY_LIMIT = 3;
            // Placeholder ring size until the origin-UPDATE trailer arrives.
            static constexpr uint32_t UNKNOWN_RING_SIZE = 1u << 20;

            // First send to `primary`, retransmits alternate to `fallback` (equal
            // args = probe). onBudgetExpired fires after retryLimit misses. Takes
            // ownership of `pkt`.
            void sendRingReliable(RingInflight& fl, nodeid_t primary, nodeid_t fallback,
                                  cPacket* pkt, simtime_t interval, int retryLimit,
                                  std::function<void()> onBudgetExpired);
            void cancelRingReliable(RingInflight& fl);
            void ringRetxFire(RingTx* h);
            std::vector<uint32_t> mySubnets;    // (subnet, myId) pairs, set at init
            simtime_t refreshInterval;
            simtime_t refreshJitter;
            simtime_t updateJitter;   // spread the initial per-ring UPDATE storm
            // Path cost = sum of member_count^ringCostExponent over the chain.
            // Integer exponent (a>=1) keeps costs in uint64 so co-best ties are
            // exact (no float ULP mismatch), which ECMP needs.
            long ringCostExponent;
            // false = rank purely by cost (oracle mode); see MyriadGp.ned.
            bool rankUseSpof = true;
            bool degradeEnabled = true;
            bool recoveryReplayEnabled = true;
            long maxEcmpPaths;   // max co-best chains kept per origin
            long maxBatchBytes;  // flush splits the held set under this size
            simtime_t holdInterval;  // batch window; 0 = send immediately
            simtime_t snapshotInterval;      // periodic RIB snapshot; 0 = off (finish() only)
            TimerMsg* snapshotTimer = nullptr;
            bool snapshotStats = false; // periodic snapshot as aggregate stats, not full RIB
            simtime_t collectWarmupDuration; // delay UPDATE origination until COLLECTs settle
            simtime_t collectWarmupInterval; // how often to re-run COLLECTs during warmup
            bool rebooted = false;
            // Held transitions bucketed by (target ring, next-hop primary).
            std::map<std::pair<uint32_t, nodeid_t>, std::vector<PendingTransition>> pendingTransitions;
            TimerMsg* holdTimer = nullptr;   // per-node; armed on the first held chain
            // Batch-size scalars for finish(); mean = entries / batchesCreated.
            unsigned long batchEntriesTotal = 0;
            unsigned long batchMaxEntries = 0;
            unsigned long batchesCreated = 0;
            uint64_t sizeToCost(uint32_t s) const;
            // Rank order: spof, then cost, then chain length. Used by both the
            // store cascade and selectPreferred.
            bool strictlyBetter(const RibEntry& a, const RibEntry& b) const;
            // Co-best: equal on (spof, cost, chain-length). Chain order is not in
            // the key, so genuine ties are retained for ECMP.
            bool coBest(const RibEntry& a, const RibEntry& b) const;
            // Trim ribIn to maxEcmpPaths, keeping the most ring-diverse subset;
            // operates on the current generation only. Returns whether candChain
            // survived.
            bool capEcmp(RingState& rs, const std::vector<uint32_t>& candChain);
            bool frontRepresented(const RingState& rs, uint32_t front, size_t except) const;
            // New-best erase that keeps each front-ring's sole current-gen.
            void dropCurGenKeepingFronts(RingState& rs, uint32_t candFront);
            // Best across all of ribIn (strictlyBetter, then lex-min chain),
            // ignoring seq — so preferred may lag curSeq (see ribIn in RingState).
            void selectPreferred(RingState& rs);

        private:
            std::unordered_map<uint32_t /*ring*/, RingState> rings;
            // Degrade state per culprit ring, kept here (not per-chain) so RIB-In
            // purges don't lose it; chains re-mark from this.
            std::unordered_map<uint32_t /*culprit ring*/,
                               std::pair<uint32_t /*seq_no*/, uint32_t /*beacon_no*/>> degrades;
            std::unordered_map<uint32_t /*subnet*/, RouteEntry> routes;
            std::unordered_map<uint32_t /*subnet*/, std::vector<uint32_t> /*ring_ids*/> subnetToRing;
            std::vector<uint32_t /*subnet*/> keys; // random destination picks in sim
    };

} // namespace routingsim

#endif
