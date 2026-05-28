#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "foundation.h"
// use kira timer for now
#include "routing/KIRA/TimerMsg.h"
#include "routing/myriadls/MyriadLsPkt_m.h"

namespace routingsim {


    enum NeighborState {
        // No connectivity to this node
        NSTATE_DOWN,
        // No relation with the neighbor yet -> send hellos
        NSTATE_INIT,
        // Two-way because my id in his hello
        NSTATE_TWOWAY,
        // Full after Exchange in both directions done
        NSTATE_FULL,
    };

    enum ExchangeInState {
        // Database description (LSDB summary) received -> send LSR
        DD_RECEIVED,
        // Link state update received -> send ack
        LSU_RECEIVED,
        // Exchange done after sending ack
        EXCHANGE_IN_DONE,
    };

    enum ExchangeOutState {
        // Database description (LSDB summary) sent -> wait for LSR
        SENDING_DD,
        // Link state update sent -> wait for ack
        LSR_RECEIVED,
        // Exchange done after receiving ack
        EXCHANGE_OUT_DONE,
    };

    struct NeighborEntry {
        nodeid_t id;
        std::string rings;
        NeighborState state = NSTATE_DOWN;
        //deadTimer
        TimerMsg* deadTimer = nullptr;
    };

    class MyriadLS : public Protocol
    {

        static constexpr int MYRIADLS_PACKET_TYPE = 55;

        protected:
            void onInitialize() override;
            void onConnect(nodeid_t nid) override;
            void onDisconnect(nodeid_t nid) override;
            void onMessage(NodePkt_p pkt) override;
            void onSelfMessage(cMessage_p msg) override;
            void reset() override;

            void scheduleHello(simtime_t delay);
            void sendHello();
            void handleHello(MyriadHello* hello, nodeid_t from);
            void handleDBD(MyriadDBD* dbd, nodeid_t from);
            void resetDead(nodeid_t nid);
            void handleDead(nodeid_t nid);

        private:
            nodeid_t myId;
            std::string rings;

            std::unordered_map<nodeid_t, NeighborEntry> neighbors;

            simtime_t helloInterval;
            simtime_t deadInterval;

            // Timer helper
            TimerMsg* addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback);


    };

    inline TimerMsg* MyriadLS::addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback) {
        auto msg = new TimerMsg(-1, callback);
        scheduleAt(simTime() + delay, msg);
        return msg;
    }

} // namespace routingsim

#endif