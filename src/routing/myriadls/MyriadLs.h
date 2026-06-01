#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "routing/myriadls/MyriadHelper.h"
#include "routing/myriadls/MyriadLsPkt_m.h"
#include <deque>

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

    // LSDB key: at most one LSA per (origin, type). Storing unique_ptr<LsaHeader>
    // relies on LsaHeader being a cObject so dup() clones the actual derived LSA.
    struct LsaKey {
        nodeid_t origin;
        MyriadLSAType type;
        bool operator==(const LsaKey& o) const { return origin == o.origin && type == o.type; }
    };

    struct LsaKeyHash {
        size_t operator()(const LsaKey& k) const noexcept {
            return std::hash<nodeid_t>()(k.origin) ^ (static_cast<size_t>(k.type) << 16);
        }
    };

    using Lsdb = std::unordered_map<LsaKey, std::unique_ptr<LsaHeader>, LsaKeyHash>;

    struct NeighborEntry {
        nodeid_t id;
        std::string rings;
        NeighborState state = NSTATE_DOWN;
        //deadTimer
        TimerMsg* deadTimer = nullptr;

        // DBD exchange state (initially both assume master)
        bool amMaster = true;
        uint16_t ddSeq = 0;
        PendingTx* pendingDbd = nullptr;

        // DBD send queue: LsaKeys we still owe this neighbor as DBD summaries.
        // Drained `dbdMaxLsasPerPacket` at a time inside sendDBD.
        std::deque<LsaKey> dbdPending;
        // Link State Request list: LSAs we want to fetch from this neighbor.
        // Stub for now — populated in a future step from received DBD summaries.
        std::deque<LsaKey> lsrPending;
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
            void installSelfLsas();
            void logLsdb();
            void buildPendingDbd(NeighborEntry& nbr);
            void logDbd(const MyriadDBD* dbd, nodeid_t from);
            void handleHello(MyriadHello* hello, nodeid_t from);
            void sendDBD(nodeid_t to, bool init, bool master, uint16_t seq_no);
            void handleDBD(MyriadDBD* dbd, nodeid_t from);
            void resetDead(nodeid_t nid);
            void handleDead(nodeid_t nid);

        private:
            std::string rings;

            std::unordered_map<nodeid_t, NeighborEntry> neighbors;
            Lsdb lsdb;

            simtime_t helloInterval;
            simtime_t deadInterval;
            simtime_t rxmtInterval;
            int dbdMaxLsasPerPacket = 10;

            // Handle to the currently-pending hello self-message so it can be cancelled on reset.
            TimerMsg* helloTimer = nullptr;
    };

} // namespace routingsim

#endif
