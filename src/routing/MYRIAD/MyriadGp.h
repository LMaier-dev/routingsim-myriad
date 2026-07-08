#ifndef __ROUTINGSIM_MYRIADGP_H_
#define __ROUTINGSIM_MYRIADGP_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadGpPkt_m.h"
#include <boost/container/flat_map.hpp>
#include <unordered_map>
#include <vector>

namespace routingsim {

    struct GpNeighbor : HelloNeighbor {
    };

    // When packet is superseeded by a newer SN, also cancel RTX and flood this one.
    struct RingInflight {
        PendingTx* tx = nullptr;
        // seq_no is bumped per collection round and does NOT reset
        // when tx clears, so stale returns from a completed round don't alias
        // a new round's inflight.
        uint32_t seq_no = 0;
        // Parallel budget timer of (retryLimit+0.5)*rxmtInterval; on fire while
        // tx still set → ring declared broken.
        TimerMsg* budgetTimer = nullptr;
    };

    struct GpPath {
        std::vector<uint32_t> chain;
        // count Single Point of Failures on path and choose the more resilient one by default,
        // tiebreaker is total length: both identical -> ECMP
        uint32_t spof_sum = 0;
    };

    struct DegradationState {
        nodeid_t culprit = 0;
        uint32_t seq_no = 0;
    };

    // Collect phase fills this for adjacent rings
    // false: single router on this ring also in `ring`
    // true: at least two routers on this ring also in `ring` (dont care for more than two)
    struct AdjacentRingState {
        uint32_t ring;
        bool isRedundant;
    };

    struct RingState {
        std::vector<MyriadGpRr> rr; // (subnet, owner) pairs collected in the latest round
        uint32_t lastUpdateSn = 0; // for supersede on incoming updates (or degrade)
        // My Rings: either collect, width, or update that's currently circulating the ring
        // Foreign Ring: update/degrade we're currently forwarding FOR THEM
        RingInflight inflight;
        std::vector<GpPath> ecmpPaths; // all paths at currentWidth (ECMP set)
        uint32_t currentWidth = 0; // width threshold defining the ECMP set
        std::vector<DegradationState> externalDegradation; // received degrade for degrade for rings on this path (could include itself)

        // derived from recent Collect lap, diff against next one
        // if different: width update this ring BEFORE emitting UPDATE
        // when forwarding: decide to sum 1 or 2 to width_sum
        std::vector<AdjacentRingState> adjacentRings;
    };

    // TODO: random numbers Myriad Construction for data and control plane

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
            // the other ring-neighbor for R. Used by plain-forward and yield.
            void insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t ring, nodeid_t from);
            // The other ring-neighbor for R (the one we did NOT receive from);
            // returns 0 if none exists.
            nodeid_t otherRingNeighbor(uint32_t ring, nodeid_t from);
            // (subnet, myId) pairs derived once at onInitialize; reused as
            // both the local-subnet source at origination and on insert.
            std::vector<uint32_t> mySubnets;

        private:
            std::unordered_map<uint32_t /*ring*/, RingState> rings;
            // Reverse lookup: subnet → ring(s) that advertise it
            std::unordered_map<uint32_t /*subnet*/, std::vector<uint32_t> /*ring_ids*/> subnetToRing;
            std::vector<uint32_t /*subnet*/> keys; // only for simulation later to pick destinations at random
    };

} // namespace routingsim

#endif
