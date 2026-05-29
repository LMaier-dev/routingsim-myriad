#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "routing/myriadls/MyriadHelper.h"
#include "routing/myriadls/MyriadLsPkt_m.h"

namespace routingsim {


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

        // DBD exchange state (initially both assume master)
        bool amMaster = true;
        // TODO add actual pending List
        bool haveMore = true;
        uint16_t ddSeq = 0;
        PendingTx* pendingDbd = nullptr;
        // DBD pending list

    };

    class MyriadLS : public MyriadHelper
    {

        static constexpr int MYRIADLS_PACKET_TYPE = 55;

        protected:
            void onInitialize() override;
            void onConnect(nodeid_t nid) override;
            void onDisconnect(nodeid_t nid) override;
            void onMessage(NodePkt_p pkt) override;
            void reset() override;

            NeighborState getNeighborState(nodeid_t nbr) const override;

            void scheduleHello(simtime_t delay);
            void sendHello();
            void handleHello(MyriadHello* hello, nodeid_t from);
            void sendDBD(nodeid_t to, bool init, bool master, bool more, uint16_t seq_no);
            void handleDBD(MyriadDBD* dbd, nodeid_t from);
            void resetDead(nodeid_t nid);
            void handleDead(nodeid_t nid);

        private:
            std::string rings;

            std::unordered_map<nodeid_t, NeighborEntry> neighbors;

            simtime_t helloInterval;
            simtime_t deadInterval;
            simtime_t rxmtInterval;
    };

} // namespace routingsim

#endif
