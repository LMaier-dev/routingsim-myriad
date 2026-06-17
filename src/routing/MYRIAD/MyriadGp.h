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
        uint32_t seq_no = 0;
    };

    struct GpPath {
        std::vector<uint32_t> chain;
        uint32_t width_sum = 0;
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
        std::vector<uint32_t> nlri; // subnets advertised in the latest update
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

        private:
            std::unordered_map<uint32_t /*ring*/, RingState> rings;
            // Reverse lookup: subnet → ring(s) that advertise it
            std::unordered_map<uint32_t /*subnet*/, std::vector<uint32_t> /*ring_ids*/> subnetToRing;
            std::vector<uint32_t /*subnet*/> keys; // only for simulation later to pick destinations at random
    };

} // namespace routingsim

#endif
