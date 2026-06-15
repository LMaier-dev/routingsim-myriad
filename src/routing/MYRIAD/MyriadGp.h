#ifndef __ROUTINGSIM_MYRIADGP_H_
#define __ROUTINGSIM_MYRIADGP_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadLsPkt_m.h"
#include <unordered_map>

namespace routingsim {

    // Path-vector variant. Shares the OSPF-style hello / DOWN-INIT-TWOWAY
    // adjacency state machine with MyriadLS via MyriadHelloProto; the rest
    // (path-vector advertisements, withdraw, loop-avoidance) lives here.
    //
    // For now this is a stub: it brings up neighbors with hellos and logs
    // when adjacencies reach TWOWAY. Fill in path-vector exchange below.
    struct GpNeighbor : HelloNeighbor {
        // TODO: per-neighbor path-vector state (advertised paths, in-flight
        // updates, withdraw queue, reliable-tx handles).
    };

    class MyriadGP : public MyriadHelloProto
    {
        // Distinct from MYRIADLS_PACKET_TYPE so both protocols can coexist.
        static constexpr int MYRIADGP_PACKET_TYPE = 56;

        protected:
            void onInitialize() override;
            void onMessage(NodePkt_p pkt) override;
            void reset() override;
            void finish() override;

            // Hello-base bridge (subclass owns the typed neighbor map).
            HelloNeighbor& findOrCreateHello(nodeid_t nid) override;
            HelloNeighbor* findHello(nodeid_t nid) override;
            const HelloNeighbor* findHello(nodeid_t nid) const override;
            void forEachHello(std::function<void(HelloNeighbor&)> fn) override;
            void forEachHello(std::function<void(const HelloNeighbor&)> fn) const override;

            void onTwoWayReached(HelloNeighbor& nbr) override;
            void onNeighborDead(HelloNeighbor& nbr, bool wasFull) override;

        private:
            std::unordered_map<nodeid_t, GpNeighbor> neighbors;
    };

} // namespace routingsim

#endif
