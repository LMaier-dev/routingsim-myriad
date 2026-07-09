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
        std::vector<RibEntry> ribIn; // all currently-valid chains for this origin
        size_t preferred = SIZE_MAX; // index into ribIn; SIZE_MAX = none

        // derived from recent Collect lap; used for spof_count on inter-ring forward
        std::vector<AdjacentRingState> adjacentRings;
    };

    // A materialized routing-table entry.
    struct RouteEntry {
        uint32_t origin_ring = 0;
        std::vector<uint32_t> chain; // prepend-order, same as RibEntry
        nodeid_t owner = 0;
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
            void handleDegrade(MyriadGpDegrade* degrade, nodeid_t from);

            //  Originate a COLLECT for ring R towards `to`.
            void originateCollect(uint32_t ring, nodeid_t to);
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
            // Forward an UPDATE within its current_ring (unconditional, for ACK).
            void forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender);
            // Transition an UPDATE into each of my other rings not yet in chain.
            void forwardUpdateInterRing(MyriadGpUpdate* pkt, uint32_t curRing);
            // Cancel a collect inflight + its budget timer for `ring`.
            void cancelCollect(uint32_t ring);
            // Budget-timer expiry for a collect on `ring`: ring declared broken.
            void onCollectBudgetExpired(uint32_t ring);
            // Budget-timer expiry for an UPDATE traversal (ring, chain).
            void onUpdateBudgetExpired(uint32_t ring, std::vector<uint32_t> chain);
            // Retransmit budget (retransmissions after the initial send).
            static constexpr int COLLECT_RETRY_LIMIT = 3;

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
