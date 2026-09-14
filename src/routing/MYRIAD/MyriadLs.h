#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "routing/MYRIAD/MyriadHello.h"
#include "routing/MYRIAD/MyriadLsPkt_m.h"
#include <deque>
#include <memory>
#include <string>

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

    // An LSA's content is fixed by (origin, type, seq) -- an origin only changes
    // what it says by bumping the seq -- so every router's copy of the same row
    // is byte-identical. Storing one object and sharing it turns the LSDB from
    // O(nodes * origins) allocations into O(origins): at n=10000 that was 3e8
    // cObjects (44GB) for 3e4 distinct LSAs. The database each router sees is
    // unchanged; only the simulator stops duplicating it.
    //
    // Const because sharing makes an installed LSA everyone's. Nothing mutates
    // one today (setters run only in buildSelf*Lsa, on a fresh object), and the
    // send paths dup() out of the LSDB before an LSU adopts the pointer.
    using LsaPtr = std::shared_ptr<const LsaHeader>;
    using Lsdb = std::unordered_map<LsaKey, LsaPtr, LsaKeyHash>;

    struct LsaPoolKey {
        nodeid_t origin;
        MyriadLSAType type;
        uint16_t seq;
        bool operator==(const LsaPoolKey& o) const {
            return origin == o.origin && type == o.type && seq == o.seq;
        }
    };

    struct LsaPoolKeyHash {
        size_t operator()(const LsaPoolKey& k) const noexcept {
            return std::hash<nodeid_t>()(k.origin)
                 ^ (static_cast<size_t>(k.type) << 16)
                 ^ (static_cast<size_t>(k.seq) << 24);
        }
    };

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
        // Slave-only: cleared at ExStart, set once we adopt the master's ddSeq.
        bool dbdStarted = false;
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

            void secCharge(const cPacket* inner) override;

            void onTwoWayReached(NeighborEntry& nbr) override;
            void onNeighborDead(NeighborEntry& nbr, bool wasFull) override;

            void installSelfLsas();
            // Debounced entry point (see doOriginateSelfLsa + minLsInterval).
            void originateSelfLsa(MyriadLSAType type, uint16_t minSeq = 0);
            void doOriginateSelfLsa(MyriadLSAType type, uint16_t minSeq);
            void logLsdb();
            // Bytes of LSDB state one router holds; see the definition.
            uint64_t lsdbBytes() const;
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
            void startSnapshot();
            void dumpOracleTo(const std::string& path, const std::string& tprefix, bool stats);

            // SPF accounting. MyriadLS forwards on the LSDB and never runs
            // Dijkstra in-protocol, so this charges what one WOULD cost to every
            // topology-LSA install -- i.e. spf-interval 0, no coalescing.
            void countSpf();
            // Ops one Dijkstra over the current ring graph would cost.
            uint64_t spfOps();
            // (relaxations, rings) of the ring graph the LSDB currently holds.
            std::pair<uint64_t, uint64_t> spfGraphSize();

        private:
            // `neighbors` (typed std::unordered_map<nodeid_t, NeighborEntry>)
            // is inherited from MyriadHelloProto<NeighborEntry>.
            Lsdb lsdb;

            int dbExchangeBatchSize = 220;
            simtime_t minLsInterval;

            // MinLSInterval debounce state, see doOriginateSelfLsa.
            std::unordered_map<LsaKey, simtime_t, LsaKeyHash> lastOriginated;
            struct PendingOriginate { uint16_t minSeq = 0; TimerMsg* timer = nullptr; };
            std::unordered_map<LsaKey, PendingOriginate, LsaKeyHash> pendingOriginate;

            // Ring-graph dimensions, recomputed only when a RINGS LSA installs
            // (membership is static once formed, so in practice: once).
            bool spfSizeDirty = true;
            uint64_t spfEdges = 0, spfRings = 0;

            simtime_t snapshotInterval;      // periodic oracle snapshot; 0 = off
            TimerMsg* snapshotTimer = nullptr;
            bool snapshotStats = false;      // periodic snapshot as aggregate stats, not full

            // The shared copy of `h`, creating it if this is the first router to
            // install that row. Weak, so a row dies with the last LSDB holding it.
            static LsaPtr intern(const LsaHeader& h);
            static std::unordered_map<LsaPoolKey, std::weak_ptr<const LsaHeader>,
                                      LsaPoolKeyHash> lsaPool;
            static size_t lsaPoolPruneAt;

            void buildSelfRingsLsa(uint16_t seq);
            void buildSelfSubnetsLsa(uint16_t seq);
            void buildSelfAdjacenciesLsa(uint16_t seq);
            void enqueueFlood(NeighborEntry& nbr, const LsaKey& key);
            void sendNextFloodBatch(NeighborEntry& nbr);
    };

} // namespace routingsim

#endif
