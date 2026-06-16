#ifndef __ROUTINGSIM_MYRIADHELLO_H_
#define __ROUTINGSIM_MYRIADHELLO_H_

#include "routing/MYRIAD/MyriadHelper.h"
#include "routing/MYRIAD/MyriadLsPkt_m.h"
#include "routing/MYRIAD/MyriadGpPkt_m.h"
#include <functional>

namespace routingsim {

    struct HelloNeighbor {
        nodeid_t id;
        NeighborState state = NSTATE_DOWN;
        TimerMsg* deadTimer = nullptr;
        std::vector<uint64_t> rings;
    };

    class MyriadHelloProto : public MyriadHelper {
        protected:
            void onInitialize() override;
            void onConnect(nodeid_t nid) override;
            void onDisconnect(nodeid_t nid) override;

            NeighborState getNeighborState(nodeid_t nbr) const override;

            virtual HelloNeighbor& findOrCreateHello(nodeid_t nid) = 0;
            // Return nullptr if not present.
            virtual HelloNeighbor* findHello(nodeid_t nid) = 0;
            virtual const HelloNeighbor* findHello(nodeid_t nid) const = 0;
            virtual void forEachHello(std::function<void(HelloNeighbor&)> fn) = 0;
            virtual void forEachHello(std::function<void(const HelloNeighbor&)> fn) const = 0;

            virtual void onTwoWayReached(HelloNeighbor& nbr) {}
            virtual void onNeighborDead(HelloNeighbor& nbr, bool wasFull) {}

            void scheduleHello(simtime_t delay);
            void sendHello();
            void handleHello(MyriadHello* hello, nodeid_t from);
            void resetDead(nodeid_t nid);
            void handleDead(nodeid_t nid);
            void cancelHelloTimer();
            std::vector<uint64_t> selfRings;
            std::string selfRingsLabel;
            void initSelfRings();

            simtime_t helloInterval;
            simtime_t deadInterval;
            simtime_t rxmtInterval;

        private:
            // Pending hello self-message (cancelled on reset).
            TimerMsg* helloTimer = nullptr;
    };

} // namespace routingsim

#endif
