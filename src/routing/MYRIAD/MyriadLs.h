#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadLsPkt_m.h"
#include <deque>
#include <memory>

namespace routingsim {

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

    // id/state/deadTimer/rings are inherited from HelloNeighbor.
    struct NeighborEntry : HelloNeighbor {
        // DBD exchange state (initially both assume master)
        bool amMaster = true;
        uint16_t ddSeq = 0;
        PendingTx* inflightDbd = nullptr;
        // At most one outstanding LSR per neighbors
        PendingTx* inflightLsr = nullptr;

        // DBD send queue: LsaKeys we still owe this neighbor as DBD summaries.
        std::deque<LsaKey> dbdQueue;
        // Link State Request list: LSAs we want to fetch from this neighbor.
        std::deque<LsaKey> lsrQueue;
        // Set when peer sent a non-init DBD with M=0.
        bool peerDbdDone = false;
        // Slave-only: cached last DBD response so we can repeat it if the
        // master retransmits its previous DBD (RFC 2328 §10.6).
        std::unique_ptr<cPacket> lastSentDbd;

        // Per-neighbor flooding queue
        std::deque<LsaKey> floodQueue;
        PendingTx* inflightFloodLsu = nullptr;
        // The seq tag in inflightFloodKeys lets us ignore stale LSAcks
        // The ones we aleady updated
        std::unordered_map<LsaKey, uint16_t, LsaKeyHash> inflightFloodKeys;
    };

    class MyriadLS : public MyriadHelloProto<NeighborEntry>
    {

        static constexpr int MYRIADLS_PACKET_TYPE = 55;

        protected:
            void onInitialize() override;
            void onMessage(NodePkt_p pkt) override;
            void reset() override;
            void finish() override;

            void onTwoWayReached(NeighborEntry& nbr) override;
            void onNeighborDead(NeighborEntry& nbr, bool wasFull) override;

            void installSelfLsas();
            void originateSelfLsa(MyriadLSAType type, uint16_t minSeq = 0);
            void logLsdb();
            void buildPendingDbd(NeighborEntry& nbr);
            void logDbd(const MyriadDBD* dbd, nodeid_t from);
            void sendDBD(nodeid_t to, bool init, bool master, uint16_t seq_no);
            void handleDBD(MyriadDBD* dbd, nodeid_t from);
            void processDBD(MyriadDBD* dbd, NeighborEntry& nbr);
            void sendNextLsr(NeighborEntry& nbr);
            void handleLSR(MyriadLSR* lsr, nodeid_t from);
            void handleLSU(MyriadLSU* lsu, nodeid_t from);
            void handleAck(MyriadAck* ack, nodeid_t from);
            void sendAck(nodeid_t to, const std::vector<const LsaHeader*>& headers);
            void tryGoFull(NeighborEntry& nbr);

        private:
            // `neighbors` (typed std::unordered_map<nodeid_t, NeighborEntry>)
            // is inherited from MyriadHelloProto<NeighborEntry>.
            Lsdb lsdb;

            int dbExchangeBatchSize = 10;

            void buildSelfRingsLsa(uint16_t seq);
            void buildSelfSubnetsLsa(uint16_t seq);
            void buildSelfAdjacenciesLsa(uint16_t seq);
            void enqueueFlood(NeighborEntry& nbr, const LsaKey& key);
            void sendNextFloodBatch(NeighborEntry& nbr);
    };

} // namespace routingsim

#endif
