#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "routing/myriadls/MyriadHelper.h"
#include "routing/myriadls/MyriadLsPkt_m.h"
#include <deque>

namespace routingsim {

    // Design simplifications vs. RFC 2328:
    //   - During initial sync (Exchange/Loading), LSU response to an LSR is
    //     fire-and-forget. Reliability is provided by the LSR retransmit timer
    //     on the requester side (one outstanding LSR per neighbor). Real OSPF
    //     acks every LSU regardless of phase; we only emit LSAcks for unsolicited
    //     (flooded) LSUs after the adjacency is FULL.
    //   - On DBD sequence mismatch we currently drop the packet; RFC §10.3
    //     prescribes a SeqNumberMismatch event that tears the adjacency back
    //     to ExStart. See TODOs in handleDBD.

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
        std::deque<LsaKey> dbdPending;
        // Link State Request list: LSAs we want to fetch from this neighbor.
        std::deque<LsaKey> lsrPending;
        // Set when peer sent a non-init DBD with M=0.
        bool peerDbdDone = false;
        // Slave-only: cached last DBD response so we can repeat it if the
        // master retransmits its previous DBD (RFC 2328 §10.6).
        cPacket* lastSentDbd = nullptr;
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
            void processDBD(MyriadDBD* dbd, NeighborEntry& nbr);
            void tryGoFull(NeighborEntry& nbr);
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
