#include "routing/MYRIAD/MyriadLs.h"
#include "routing/MYRIAD/RingSpfOracle.h"
#include "routing/MYRIAD/MyriadPduSize.h"
#include <algorithm>
#include <set>

namespace routingsim {

Define_Module(MyriadLS);

std::unordered_map<LsaPoolKey, std::weak_ptr<const LsaHeader>,
                   LsaPoolKeyHash> MyriadLS::lsaPool;
size_t MyriadLS::lsaPoolPruneAt = 1024;

namespace {
using namespace msgsize;

int64_t lsaBytes(const LsaHeader* h) {
    int64_t b = LSA_HDR;
    switch (h->getType()) {
        case LSA_ADJACENCIES:
            b += (int64_t) check_and_cast<const AdjacenciesLsa*>(h)->getAdjacenciesArraySize() * NODEID; break;
        case LSA_SUBNETS:
            b += (int64_t) check_and_cast<const SubnetsLsa*>(h)->getSubnetsArraySize() * U32; break;
        case LSA_RINGS:
            b += (int64_t) check_and_cast<const RingsLsa*>(h)->getRingsArraySize() * RING; break;
    }
    return b;
}
// Same row, same content? (origin, type, seq) is normally enough to answer yes,
// but a rebooted node restarts its sequence at 1 and may say something different
// there than it did before, so the pool verifies rather than assumes.
bool sameLsa(const LsaHeader& a, const LsaHeader& b) {
    if (a.getType() != b.getType() || a.getSeq_no() != b.getSeq_no()
            || a.getLsa_origin() != b.getLsa_origin())
        return false;
    switch (a.getType()) {
        case LSA_ADJACENCIES: {
            auto &x = *check_and_cast<const AdjacenciesLsa*>(&a);
            auto &y = *check_and_cast<const AdjacenciesLsa*>(&b);
            if (x.getAdjacenciesArraySize() != y.getAdjacenciesArraySize()) return false;
            for (size_t i = 0; i < x.getAdjacenciesArraySize(); ++i)
                if (!(x.getAdjacencies(i) == y.getAdjacencies(i))) return false;
            return true;
        }
        case LSA_SUBNETS: {
            auto &x = *check_and_cast<const SubnetsLsa*>(&a);
            auto &y = *check_and_cast<const SubnetsLsa*>(&b);
            if (x.getSubnetsArraySize() != y.getSubnetsArraySize()) return false;
            for (size_t i = 0; i < x.getSubnetsArraySize(); ++i)
                if (x.getSubnets(i) != y.getSubnets(i)) return false;
            return true;
        }
        case LSA_RINGS: {
            auto &x = *check_and_cast<const RingsLsa*>(&a);
            auto &y = *check_and_cast<const RingsLsa*>(&b);
            if (x.getRingsArraySize() != y.getRingsArraySize()) return false;
            for (size_t i = 0; i < x.getRingsArraySize(); ++i)
                if (x.getRings(i) != y.getRings(i)) return false;
            return true;
        }
    }
    return false;
}
int64_t lsuBytes(const MyriadLSU* p) {
    int64_t b = MYRIADLS_HDR;
    for (size_t i = 0; i < p->getLsasArraySize(); ++i) b += lsaBytes(p->getLsas(i));
    return b;
}
int64_t lsrBytes(const MyriadLSR* p) {
    return MYRIADLS_HDR + (int64_t) p->getReqsArraySize() * LSREQ;
}
int64_t ackBytes(const MyriadAck* p) {
    return MYRIADLS_HDR + (int64_t) p->getAckedArraySize() * LSA_HDR;
}
int64_t dbdBytes(const MyriadDBD* p) {
    return MYRIADLS_HDR + SEQ + 3          // seq_no + init/master/more bools
         + (int64_t) p->getLs_dbdsArraySize() * LSA_HDR;
}
} // namespace

void MyriadLS::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADLS_PACKET_TYPE;
    dbExchangeBatchSize = par("dbExchangeBatchSize");
    minLsInterval = par("minLsInterval");
    snapshotInterval = par("snapshotInterval");
    snapshotStats = par("snapshotFormat").stdstringValue() == "stats";

    node().bind(MYRIADLS_PACKET_TYPE, this);

    txBytesSecVec = new cOutVector("txBytesSec");
    txBytesSecPerHopVec = new cOutVector("txBytesSecPerHop");
    secLinkPerPdu = msgsize::SEC_LINK_MAC;
    secLinkHello  = msgsize::SEC_LINK_SIG + msgsize::SEC_HELLO;

    installSelfLsas();

    // periodically send hellos to all neighbors
    scheduleHello(helloInterval + uniform(0, joinJitter));

    if (snapshotInterval > 0 && !par("oracleDumpFile").stdstringValue().empty())
        startSnapshot();
}

void MyriadLS::onTwoWayReached(NeighborEntry& nbr) {
    nbr.ddSeq = 0; // arbitrary starting seq no based on node ids
    nbr.amMaster = true;
    nbr.dbdStarted = false;
    buildPendingDbd(nbr);
    sendDBD(nbr.id, /*init=*/true, nbr.amMaster, nbr.ddSeq);
}

void MyriadLS::onNeighborDead(NeighborEntry& nbr, bool wasFull) {
    cancelReliable(nbr.inflightDbd);
    cancelReliable(nbr.inflightLsr);
    cancelReliable(nbr.inflightFloodLsu);
    nbr.inflightFloodKeys.clear();
    nbr.floodQueue.clear();
    nbr.amMaster = false;
    nbr.ddSeq = 0;
    nbr.dbdStarted = false;
    nbr.dbdQueue.clear();
    nbr.lsrQueue.clear();
    nbr.peerDbdDone = false;
    nbr.lastSentDbd.reset();

    // Lost a FULL adjacency: re-originate AdjacenciesLsa
    if (wasFull) originateSelfLsa(LSA_ADJACENCIES);
}

void MyriadLS::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    auto header = std::unique_ptr<MyriadLsHeader>(check_and_cast<MyriadLsHeader*>(pkt->decapsulate()));
    nodeid_t from = header->getRouter_id_src();
    switch (header->getType()) {
        case MYRIAD_HELLO:
            handleHello(check_and_cast<MyriadHello*>(header.get()), from);
            break;
        case MYRIAD_DBD:
            handleDBD(check_and_cast<MyriadDBD*>(header.get()), from);
            break;
        case MYRIAD_LSR:
            handleLSR(check_and_cast<MyriadLSR*>(header.get()), from);
            break;
        case MYRIAD_LSU:
            handleLSU(check_and_cast<MyriadLSU*>(header.get()), from);
            break;
        case MYRIAD_ACK:
            handleAck(check_and_cast<MyriadAck*>(header.get()), from);
            break;
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }

}

// A router crash: NetworkManager calls this when the node fails. The LSDB is
// lost, so the returning router re-originates its own LSAs and refills the rest
// from the database exchange its neighbors start once hellos reach TWOWAY again.
void MyriadLS::reset() {
    Enter_Method_Silent();   // invoked cross-module from NetworkManager; we reschedule below
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.deadTimer) {
            cancelAndDelete(nbr.deadTimer);
            nbr.deadTimer = nullptr;
        }
        cancelReliable(nbr.inflightDbd);
        cancelReliable(nbr.inflightLsr);
        cancelReliable(nbr.inflightFloodLsu);
        nbr.inflightFloodKeys.clear();
        nbr.floodQueue.clear();
        nbr.lastSentDbd.reset();
    }
    neighbors.clear();
    lsdb.clear();
    spfSizeDirty = true;
    for (auto& [key, pend] : pendingOriginate) cancelAndDelete(pend.timer);
    pendingOriginate.clear();
    lastOriginated.clear();
    installSelfLsas();

    if (snapshotTimer) { cancelAndDelete(snapshotTimer); snapshotTimer = nullptr; }
    cancelHelloTimer();

    scheduleHello(helloInterval);
    if (snapshotInterval > 0 && !par("oracleDumpFile").stdstringValue().empty())
        startSnapshot();
}


LsaPtr MyriadLS::intern(const LsaHeader& h) {
    LsaPoolKey k{h.getLsa_origin(), h.getType(), h.getSeq_no()};
    auto& slot = lsaPool[k];
    if (auto sp = slot.lock())
        if (sameLsa(*sp, h)) return sp;      // hit: every holder shares this row

    LsaPtr sp(check_and_cast<LsaHeader*>(h.dup()));
    slot = sp;

    // ponytail: entries whose last holder is gone linger; sweep when the table
    // has doubled, which bounds it at ~2x live. A generation counter per row
    // would be exact, if the sweep ever shows up in a profile.
    if (lsaPool.size() > lsaPoolPruneAt) {
        for (auto it = lsaPool.begin(); it != lsaPool.end(); )
            it = it->second.expired() ? lsaPool.erase(it) : std::next(it);
        lsaPoolPruneAt = 2 * lsaPool.size() + 1024;
    }
    return sp;
}

void MyriadLS::buildSelfRingsLsa(uint16_t seq) {
    auto lsa = std::make_unique<RingsLsa>();
    lsa->setSeq_no(seq);
    lsa->setLsa_origin(myId);
    lsa->setRingsArraySize(selfRings.size());
    for (size_t i = 0; i < selfRings.size(); ++i) lsa->setRings(i, selfRings[i]);

    lsdb[LsaKey{myId, LSA_RINGS}] = intern(*lsa);
}

void MyriadLS::buildSelfSubnetsLsa(uint16_t seq) {
    auto lsa = std::make_unique<SubnetsLsa>();
    lsa->setSeq_no(seq);
    lsa->setLsa_origin(myId);
    // Bogus subnets derived from node id: <id>.1 and <id>.2
    lsa->setSubnetsArraySize(2);
    lsa->setSubnets(0, (static_cast<uint32_t>(myId) << 8) | 0x01);
    lsa->setSubnets(1, (static_cast<uint32_t>(myId) << 8) | 0x02);

    lsdb[LsaKey{myId, LSA_SUBNETS}] = intern(*lsa);
}

void MyriadLS::buildSelfAdjacenciesLsa(uint16_t seq) {
    std::vector<nodeid_t> adj;
    for (const auto& [nid, nbr] : neighbors) {
        if (nbr.state == NSTATE_FULL) adj.push_back(nid);
    }

    auto lsa = std::make_unique<AdjacenciesLsa>();
    lsa->setSeq_no(seq);
    lsa->setLsa_origin(myId);
    lsa->setAdjacenciesArraySize(adj.size());
    for (size_t i = 0; i < adj.size(); ++i) lsa->setAdjacencies(i, adj[i]);

    lsdb[LsaKey{myId, LSA_ADJACENCIES}] = intern(*lsa);
}

void MyriadLS::installSelfLsas() {
    originateSelfLsa(LSA_RINGS);
    originateSelfLsa(LSA_SUBNETS);
    originateSelfLsa(LSA_ADJACENCIES);
}

void MyriadLS::originateSelfLsa(MyriadLSAType type, uint16_t minSeq) {
    LsaKey key{myId, type};
    simtime_t now = simTime();
    auto lit = lastOriginated.find(key);
    if (lit == lastOriginated.end() || now - lit->second >= minLsInterval) {
        doOriginateSelfLsa(type, minSeq);
        return;
    }
    // Within the debounce window (OSPF MinLSInterval): coalesce bursts (e.g.
    // many neighbors reaching FULL together, or a high-degree hub) into one
    // re-origination scheduled at the window edge, instead of one per event.
    auto& pend = pendingOriginate[key];
    if (minSeq > pend.minSeq) pend.minSeq = minSeq;
    if (pend.timer) return; // already scheduled
    simtime_t delay = minLsInterval - (now - lit->second);
    pend.timer = addOneShotTimer(delay, [this, type, key](short) {
        uint16_t seq = pendingOriginate[key].minSeq;
        pendingOriginate.erase(key);
        doOriginateSelfLsa(type, seq);
        return false;
    });
}

void MyriadLS::doOriginateSelfLsa(MyriadLSAType type, uint16_t minSeq) {
    LsaKey key{myId, type};
    lastOriginated[key] = simTime();
    auto it = lsdb.find(key);
    uint16_t newSeq = (it != lsdb.end()) ? it->second->getSeq_no() + 1 : 1;
    if (static_cast<uint16_t>(minSeq + 1) > newSeq) newSeq = minSeq + 1;

    switch (type) {
        case LSA_RINGS:       buildSelfRingsLsa(newSeq);       break;
        case LSA_SUBNETS:     buildSelfSubnetsLsa(newSeq);     break;
        case LSA_ADJACENCIES: buildSelfAdjacenciesLsa(newSeq); break;
    }

    MLOG_INFO() << "originated self-LSA type=" << type << " seq=" << newSeq << "\n";

    // Own topology change counts too: a router recomputes on its own lost
    // adjacency rather than waiting to hear it back from the flood.
    if (type != LSA_SUBNETS) countSpf();
    if (type == LSA_RINGS) spfSizeDirty = true;

    // Flood to TWOWAY+ neighbors (OSPF's Exchange+): an LSA originated mid-exchange
    // must reach a neighbor whose DBD summary was already snapshotted.
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.state < NSTATE_TWOWAY) continue;
        enqueueFlood(nbr, key);
        sendNextFloodBatch(nbr);
    }
}

void MyriadLS::buildPendingDbd(NeighborEntry& nbr) {
    nbr.dbdQueue.clear();
    for (const auto& kv : lsdb) {
        nbr.dbdQueue.push_back(kv.first);
    }
}

void MyriadLS::logDbd(const MyriadDBD* dbd, nodeid_t from) {
    if (!logEnabled(MLOG_LEVEL_DEBUG)) return;
    MLOG_N_DEBUG(from) << "DBD seq=" << dbd->getSeq_no()
                       << " I=" << dbd->getInit()
                       << " M=" << dbd->getMore()
                       << " MS=" << dbd->getMaster()
                       << " entries=" << dbd->getLs_dbdsArraySize() << "\n";
    for (size_t i = 0; i < dbd->getLs_dbdsArraySize(); ++i) {
        const LsaHeader& h = dbd->getLs_dbds(i);
        MLOG_N_DEBUG(from) << "  origin=" << (int)h.getLsa_origin()
                           << " type=" << h.getType()
                           << " seq=" << h.getSeq_no() << "\n";
    }
}

void MyriadLS::logLsdb() {
    if (!logEnabled(MLOG_LEVEL_INFO)) return;
    MLOG_INFO() << "LSDB (" << lsdb.size() << " entries):\n";
    for (const auto& [key, lsa] : lsdb) {
        MLOG_INFO() << "  origin=" << (int)key.origin
                    << " type=" << key.type
                    << " seq=" << lsa->getSeq_no();
        switch (lsa->getType()) {
            case LSA_RINGS: {
                auto* r = check_and_cast<const RingsLsa*>(lsa.get());
                EV << " rings=[";
                for (size_t i = 0; i < r->getRingsArraySize(); ++i)
                    EV << (i ? "," : "") << (char)r->getRings(i);
                EV << "]";
                break;
            }
            case LSA_SUBNETS: {
                auto* s = check_and_cast<const SubnetsLsa*>(lsa.get());
                EV << " subnets=[";
                for (size_t i = 0; i < s->getSubnetsArraySize(); ++i)
                    EV << (i ? "," : "") << s->getSubnets(i);
                EV << "]";
                break;
            }
            case LSA_ADJACENCIES: {
                auto* a = check_and_cast<const AdjacenciesLsa*>(lsa.get());
                EV << " adjacencies=[";
                for (size_t i = 0; i < a->getAdjacenciesArraySize(); ++i)
                    EV << (i ? "," : "") << (int)a->getAdjacencies(i);
                EV << "]";
                break;
            }
        }
        EV << "\n";
    }
}

void MyriadLS::processDBD(MyriadDBD* dbd, NeighborEntry& nbr) {
    // iterate over dbds compare to lsdb with the key and add newer seq no to the lsr_pending
    for (size_t i = 0; i < dbd->getLs_dbdsArraySize(); ++i) {
        const LsaHeader& h = dbd->getLs_dbds(i);
        if (h.getLsa_origin() == myId) continue;
        LsaKey key{h.getLsa_origin(), h.getType()};
        auto it = lsdb.find(key);
        // dont insert if our LSDB is at least same age
        if (it != lsdb.end() && it->second->getSeq_no() >= h.getSeq_no()) continue;
        // dont insert if its already there
        if (std::find(nbr.lsrQueue.begin(), nbr.lsrQueue.end(), key) != nbr.lsrQueue.end()) continue;
        MLOG_N_DEBUG(nbr.id) << "LSDB missing or stale for "
                << (int)key.origin << "/" << key.type << ", queued for LSR\n";
        nbr.lsrQueue.push_back(key);
    }
    // Requests can be sent concurrently with the DBD exchange.
    sendNextLsr(nbr);
}

void MyriadLS::sendNextLsr(NeighborEntry& nbr) {
    if (nbr.lsrQueue.empty()) {
        // satisfied the last outstanding requests.
        // But go full only if no DBDs exchange finished
        tryGoFull(nbr);
        return;
    }
    // At most one inflight LSR per neighbor.
    if (nbr.inflightLsr) return;

    auto* lsr = new MyriadLSR("Myriad_LSR");
    lsr->setRouter_id_src(myId);

    size_t batch = std::min(nbr.lsrQueue.size(), static_cast<size_t>(dbExchangeBatchSize));
    lsr->setReqsArraySize(batch);
    auto it = nbr.lsrQueue.begin();
    for (size_t i = 0; i < batch; ++i, ++it) {
        LsaRequest r;
        r.origin = it->origin;
        r.type = it->type;
        lsr->setReqs(i, r);
    }
    MLOG_N_DEBUG(nbr.id) << "sending LSR with " << batch << " request(s)\n";
    lsr->setByteLength(lsrBytes(lsr));
    nbr.inflightLsr = sendReliable(nbr.id, lsr, rxmtInterval);
}

void MyriadLS::handleLSR(MyriadLSR* lsr, nodeid_t from) {
    auto* lsu = new MyriadLSU("Myriad_LSU");
    lsu->setRouter_id_src(myId);

    size_t n = lsr->getReqsArraySize();
    std::vector<LsaHeader*> dups;
    dups.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const LsaRequest& r = lsr->getReqs(i);
        LsaKey key{r.origin, r.type};
        auto it = lsdb.find(key);
        if (it == lsdb.end()) {
            // TODO: BadLSReq — should reset the adjacency.
            MLOG_N_WARN(from) << "received LSR for unknown LSA "
                              << (int)key.origin << "/" << key.type << "\n";
            continue;
        }
        dups.push_back(check_and_cast<LsaHeader*>(it->second->dup()));
    }

    lsu->setLsasArraySize(dups.size());
    for (size_t i = 0; i < dups.size(); ++i) {
        lsu->setLsas(i, dups[i]); // @owned: LSU adopts the pointer
    }

    MLOG_N_DEBUG(from) << "sending LSU with " << dups.size() << " LSA(s)\n";
    auto np = std::make_unique<NodePkt>("Myriad_LSU_Pkt");
    np->setType(MYRIADLS_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    lsu->setByteLength(lsuBytes(lsu));
    np->encapsulate(lsu);
    secCharge(lsu);
    countSend("lsu", np->getByteLength());
    sendToNode(from, std::move(np));
}

void MyriadLS::handleLSU(MyriadLSU* lsu, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    size_t n = lsu->getLsasArraySize();
    MLOG_N_DEBUG(from) << "received LSU with " << n << " LSA(s)\n";

    bool drainedAny = false;
    std::vector<const LsaHeader*> toAck;
    std::vector<LsaKey> toReflood;

    for (size_t i = 0; i < n; ++i) {
        const LsaHeader* h = lsu->getLsas(i);
        LsaKey key{h->getLsa_origin(), h->getType()};
        bool isSelf = (h->getLsa_origin() == myId);
        bool installed = false;

        if (isSelf) {
            // Self-LSA loopback: someone propagated a higher-SN
            // Re-originate fresh from current state with bigger SN
            auto it = lsdb.find(key);
            if (it != lsdb.end() && h->getSeq_no() > it->second->getSeq_no()) {
                MLOG_N_INFO(from) << "self-LSA loopback for " << (int)key.origin
                                  << "/" << key.type << " received seq=" << h->getSeq_no()
                                  << "; re-originating\n";
                originateSelfLsa(key.type, h->getSeq_no());
            }
        } else {
            auto it = lsdb.find(key);
            if (it == lsdb.end() || it->second->getSeq_no() < h->getSeq_no()) {
                lsdb[key] = intern(*h);
                installed = true;
                if (key.type != LSA_SUBNETS) countSpf();  // topology changed
                if (key.type == LSA_RINGS) spfSizeDirty = true;
                MLOG_N_DEBUG(from) << "installed LSA " << (int)key.origin
                                   << "/" << key.type << " seq=" << h->getSeq_no() << "\n";
            }
        }

        // Ack every received LSA so flooding can overlap the DB exchange:
        // a flood needs the ack, an LSR response ignores the spurious one.
        // Drain the request if this satisfies it; reflood any newer install.
        auto pit = std::find(nbr.lsrQueue.begin(), nbr.lsrQueue.end(), key);
        if (pit != nbr.lsrQueue.end()) {
            nbr.lsrQueue.erase(pit);
            drainedAny = true;
        }
        toAck.push_back(h);
        if (installed) toReflood.push_back(key);
    }

    // Reflood newly-installed LSAs to TWOWAY+ neighbors except the sender.
    if (!toReflood.empty()) {
        for (auto& [nid, nb] : neighbors) {
            if (nb.state < NSTATE_TWOWAY) continue;
            if (nid == from) continue;
            for (const auto& k : toReflood) enqueueFlood(nb, k);
            sendNextFloodBatch(nb);
        }
    }

    // pass pointer to packtet for slice reply
    if (!toAck.empty()) sendAck(from, toAck);

    if (drainedAny) {
        cancelReliable(nbr.inflightLsr);
        sendNextLsr(nbr);
    }
}

void MyriadLS::enqueueFlood(NeighborEntry& nbr, const LsaKey& key) {
    // Dedup against the queue
    if (std::find(nbr.floodQueue.begin(), nbr.floodQueue.end(), key) != nbr.floodQueue.end()) return;
    nbr.floodQueue.push_back(key);
}

void MyriadLS::sendNextFloodBatch(NeighborEntry& nbr) {
    if (nbr.inflightFloodLsu != nullptr) return; // wait for current batch's ack
    if (nbr.floodQueue.empty()) return;

    auto* lsu = new MyriadLSU("Myriad_LSU_Flood");
    lsu->setRouter_id_src(myId);

    size_t batch = std::min(nbr.floodQueue.size(), static_cast<size_t>(dbExchangeBatchSize));
    std::vector<LsaHeader*> dups;
    dups.reserve(batch);
    nbr.inflightFloodKeys.clear();
    for (size_t i = 0; i < batch; ++i) {
        LsaKey key = nbr.floodQueue.front();
        nbr.floodQueue.pop_front();
        auto lit = lsdb.find(key);
        if (lit == lsdb.end()) continue; // defensive: LSA dropped from LSDB
        dups.push_back(check_and_cast<LsaHeader*>(lit->second->dup()));
        nbr.inflightFloodKeys[key] = lit->second->getSeq_no();
    }

    if (dups.empty()) {
        delete lsu;
        return;
    }

    lsu->setLsasArraySize(dups.size());
    for (size_t i = 0; i < dups.size(); ++i) lsu->setLsas(i, dups[i]); // @owned

    lsu->setByteLength(lsuBytes(lsu));
    nbr.inflightFloodLsu = sendReliable(nbr.id, lsu, rxmtInterval);
    MLOG_N_DEBUG(nbr.id) << "flooding LSU with " << dups.size() << " LSA(s)\n";
}

void MyriadLS::sendAck(nodeid_t to, const std::vector<const LsaHeader*>& headers) {
    auto* ack = new MyriadAck("Myriad_LSAck");
    ack->setRouter_id_src(myId);
    ack->setAckedArraySize(headers.size());
    // slice only header fields
    for (size_t i = 0; i < headers.size(); ++i) ack->setAcked(i, *headers[i]);

    auto np = std::make_unique<NodePkt>("Myriad_LSAck_Pkt");
    np->setType(MYRIADLS_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    ack->setByteLength(ackBytes(ack));
    np->encapsulate(ack);
    countSend("ack", np->getByteLength());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "sent LSAck for " << headers.size() << " LSA(s)\n";
}

void MyriadLS::handleAck(MyriadAck* ack, nodeid_t from) {
    auto it = neighbors.find(from);
    if (it == neighbors.end()) return;
    NeighborEntry& nbr = it->second;

    size_t n = ack->getAckedArraySize();
    MLOG_N_DEBUG(from) << "received LSAck for " << n << " LSA(s)\n";

    for (size_t i = 0; i < n; ++i) {
        const LsaHeader& h = ack->getAcked(i);
        LsaKey key{h.getLsa_origin(), h.getType()};
        auto kit = nbr.inflightFloodKeys.find(key);
        if (kit == nbr.inflightFloodKeys.end()) continue;
        // Ignore stale acks for an LSA we've since superseded.
        if (kit->second != h.getSeq_no()) continue;
        nbr.inflightFloodKeys.erase(kit);
    }

    // free to send next batch if we acked all LSAs in the current batch
    if (nbr.inflightFloodKeys.empty() && nbr.inflightFloodLsu != nullptr) {
        cancelReliable(nbr.inflightFloodLsu);
        sendNextFloodBatch(nbr);
    }
}

void MyriadLS::tryGoFull(NeighborEntry& nbr) {
    if (nbr.state == NSTATE_FULL) return;
    if (!nbr.dbdQueue.empty()) return;
    if (nbr.inflightDbd != nullptr) return;
    if (!nbr.peerDbdDone) return;
    if (!nbr.lsrQueue.empty()) return;
    if (nbr.inflightLsr != nullptr) return;
    nbr.state = NSTATE_FULL;
    MLOG_N_INFO(nbr.id) << "going FULL\n";
    originateSelfLsa(LSA_ADJACENCIES);
}

void MyriadLS::dumpOracleTo(const std::string& path, const std::string& tprefix, bool stats) {
    // Rebuild per-node ring membership from the flooded RingsLsa, then let the
    // oracle run Dijkstra over the ring graph.
    std::map<ringspf::NodeId, std::vector<ringspf::RingId>> perNodeRings;
    for (const auto& [key, lsa] : lsdb) {
        if (key.type != LSA_RINGS) continue;
        auto* r = check_and_cast<const RingsLsa*>(lsa.get());
        auto& rs = perNodeRings[(ringspf::NodeId)key.origin];
        for (size_t i = 0; i < r->getRingsArraySize(); ++i)
            rs.push_back(r->getRings(i));
    }

    // A RingsLsa is a membership declaration and nothing withdraws it, so a dead
    // node stays in the LSDB. Two uses, two bases:
    //   - present count: keep the full membership. A route lingers until
    //     reconvergence, matching a path-vector RIB's stale entries, so the
    //     route-refill axis compares like with like.
    //   - cost: prune to what is still reachable from here (Adjacency LSAs track
    //     liveness — a survivor re-originates without the dead peer), or the
    //     oracle would price the pre-failure topology. The full view is still
    //     what ring WEIGHTS are counted over.
    const auto allRings = perNodeRings;
    long presentCount =
        (long) ringspf::shortestRingCostsFor(perNodeRings,
            par("ringCostExponent"), par("echoRingSize"), (ringspf::NodeId)myId).size();

    std::map<nodeid_t, std::vector<nodeid_t>> adj;
    for (const auto& [key, lsa] : lsdb) {
        if (key.type != LSA_ADJACENCIES) continue;
        auto* a = check_and_cast<const AdjacenciesLsa*>(lsa.get());
        auto& v = adj[key.origin];
        for (size_t i = 0; i < a->getAdjacenciesArraySize(); ++i)
            v.push_back(a->getAdjacencies(i));
    }
    std::set<nodeid_t> live{myId};
    for (std::vector<nodeid_t> q{myId}; !q.empty(); ) {
        nodeid_t n = q.back(); q.pop_back();
        auto it = adj.find(n);
        if (it == adj.end()) continue;
        for (nodeid_t p : it->second)
            if (live.insert(p).second) q.push_back(p);
    }
    for (auto it = perNodeRings.begin(); it != perNodeRings.end(); )
        it = live.count((nodeid_t)it->first) ? std::next(it) : perNodeRings.erase(it);
    ringspf::dumpOracle(path, (ringspf::NodeId)myId, perNodeRings,
                        par("ringCostExponent"), par("echoRingSize"), tprefix, stats,
                        presentCount, &allRings);
}

std::pair<uint64_t, uint64_t> MyriadLS::spfGraphSize() {
    if (spfSizeDirty) {
        // The graph RingSpfOracle builds
        std::set<uint32_t> ringSet;
        uint64_t edges = 0;
        for (const auto& [key, lsa] : lsdb) {
            if (key.type != LSA_RINGS) continue;
            auto* r = check_and_cast<const RingsLsa*>(lsa.get());
            size_t k = r->getRingsArraySize();
            for (size_t i = 0; i < k; ++i) ringSet.insert(r->getRings(i));
            edges += (uint64_t) k * (k - 1);   // ordered pairs = 2 * C(k,2)
        }
        spfRings = ringSet.size();
        spfEdges = edges;
        spfSizeDirty = false;
    }
    return {spfEdges, spfRings};
}

// Binary-heap Dijkstra over the current ring graph: a relaxation per edge, a
// sift per settled vertex
uint64_t MyriadLS::spfOps() {
    auto [e, r] = spfGraphSize();
    if (!r) return 0;
    uint64_t logr = 0;
    for (uint64_t v = r; v > 1; v >>= 1) ++logr;
    return e + r * (logr + 1);
}

// One SPF charged per topology-LSA install: spf-interval 0, the upper bound.
// Real OSPF coalesces a failure's burst into one run, but how much it coalesces
// swings the total by ~20x, eval replays and sets window itself.
void MyriadLS::countSpf() {
    if (uint64_t ops = spfOps()) countCalc(ops);
}

void MyriadLS::startSnapshot() {
    // Re-arms itself like the hello timer; purely observational (see header).
    snapshotTimer = addOneShotTimer(snapshotInterval, [this](short) {
        snapshotTimer = nullptr;
        dumpOracleTo(par("oracleDumpFile").stdstringValue(),
                     "t=" + simTime().str() + " ", snapshotStats);
        startSnapshot();
        return false;
    });
}

// an LSA floods network-wide, so the hop-by-hop MAC does not reach far enough to authenticate it. 
// DBD, LSR and ACK carry only LSA headers and never leave the link, so the MAC is all they need.
void MyriadLS::secCharge(const cPacket* inner) {
    auto* u = dynamic_cast<const MyriadLSU*>(inner);
    int64_t n = u ? (int64_t) u->getLsasArraySize() : 0;
    secSigBytes += n * msgsize::SEC_SIG;
    secSigPerHopBytes += n * msgsize::SEC_SIG_PERHOP;
    secPduExtra = n * msgsize::SEC_SIG;
    secPduExtraPerHop = n * msgsize::SEC_SIG_PERHOP;
}

void MyriadLS::finish() {
    logLsdb();
    recordSendScalars();
    recordCalcScalars();
    recordSecScalars();
    recordScalar("state_bytes", (double) lsdbBytes());
    // The per-run SPF input, so calc_ops reads as "N Dijkstras over R rings".
    auto [e, r] = spfGraphSize();
    recordScalar("spf_rings", (double) r);
    recordScalar("spf_edges", (double) e);

    // Optimality oracle (see RingSpfOracle)
    std::string oracleFile = par("oracleDumpFile").stdstringValue();
    if (!oracleFile.empty())
        dumpOracleTo(oracleFile, snapshotInterval > 0 ? "t=" + simTime().str() + " " : "", false);
}

// Protocol state THIS router holds, at the wire field widths of MyriadPduSize.h
// so it is commensurable with bytes_total and with PV's state_bytes.
uint64_t MyriadLS::lsdbBytes() const {
    uint64_t b = 0;
    for (const auto& [key, lsa] : lsdb) {
        b += msgsize::LSA_HDR;
        switch (lsa->getType()) {
            case LSA_ADJACENCIES:
                b += check_and_cast<const AdjacenciesLsa*>(lsa.get())
                         ->getAdjacenciesArraySize() * msgsize::NODEID;
                break;
            case LSA_SUBNETS:
                b += check_and_cast<const SubnetsLsa*>(lsa.get())
                         ->getSubnetsArraySize() * msgsize::U32;
                break;
            case LSA_RINGS:
                b += check_and_cast<const RingsLsa*>(lsa.get())
                         ->getRingsArraySize() * msgsize::RING;
                break;
        }
    }
    return b;
}

void MyriadLS::handleDBD(MyriadDBD* dbd, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    logDbd(dbd, from);
    // Before our own ExStart the DBD queue isn't built yet; acting on a peer's
    // DBD here would answer with an empty summary and then be wiped by ExStart.
    if (nbr.state < NSTATE_TWOWAY) {
        MLOG_N_DEBUG(from) << "ignoring DBD before TWO-WAY\n";
        return;
    }
    if (myId > from) { // master
        if (dbd->getInit()) { // Phase 1 master negotioation
            if (nbr.ddSeq != 0) {
                // Exchange already started: this is a stale slave-init retransmit
                MLOG_N_DEBUG(from) << "ignoring stale slave init after exchange start\n";
            } else {
                nbr.amMaster = true;
                nbr.ddSeq = nbr.ddSeq + 1;
                // Phase 2 intiated by master
                cancelReliable(nbr.inflightDbd);
                sendDBD(from, /*init=*/false, nbr.amMaster, nbr.ddSeq);
            }
        } else {
            if (!dbd->getMaster() && dbd->getSeq_no() == nbr.ddSeq && nbr.state < NSTATE_FULL) {
                processDBD(dbd, nbr); // expected DBD, process and respond with next one
                cancelReliable(nbr.inflightDbd);
                MLOG_N_DEBUG(from) << "received expected DBD from slave, seq=" << dbd->getSeq_no() << "\n";
                if (!dbd->getMore()) nbr.peerDbdDone = true;
                nbr.ddSeq++; // master increments before response
                // If both sides are out of LSAs, skip the empty follow-up DBD.
                if (!(nbr.peerDbdDone && nbr.dbdQueue.empty())) {
                    sendDBD(from, /*init=*/false, nbr.amMaster, nbr.ddSeq);
                }
                tryGoFull(nbr);
            } else if (!dbd->getMaster() && nbr.state == NSTATE_FULL) {
                MLOG_N_DEBUG(from) << "received additional DBD from slave after FULL, seq=" << dbd->getSeq_no() << "\n";
            } else {
                // TODO: Generate SeqNumberMismatch: clear DBD/LSR
                // state and restart from ExStart. Currently we just drop the packet.
                MLOG_N_WARN(from) << "received unexpected DBD from slave\n";
            }
        }
    } else { // slave
        if (dbd->getInit()) { // Phase 1 master negotioation
            nbr.amMaster = false;
            nbr.ddSeq = dbd->getSeq_no() + 1;
            // Keep our own init retransmitting (do NOT cancel it here). The master
            // starts the exchange only from its getInit handler, so it must receive
            // our init AFTER it reaches TWO-WAY — otherwise a staggered join, where
            // the master ignored our earlier inits, deadlocks in ExStart. The init
            // is cancelled once the master's phase-2 arrives (the else branch below).
        } else {
            // slaves never retransmit (cancel just in case, but it should be null)
            cancelReliable(nbr.inflightDbd);
            // Phase 2 response by slave
            if (dbd->getMaster() && (dbd->getSeq_no() == nbr.ddSeq || !nbr.dbdStarted)
                    && nbr.state < NSTATE_FULL) {
                // The master owns the sequence: adopt it on the exchange's first
                // DBD instead of insisting on the value our own init negotiated,
                // which is stale whenever the master ExStarted before we did.
                nbr.amMaster = false;
                nbr.ddSeq = dbd->getSeq_no();
                nbr.dbdStarted = true;
                processDBD(dbd, nbr); // expected DBD, process and respond with next one
                MLOG_N_DEBUG(from) << "received expected DBD from master, seq=" << dbd->getSeq_no() << "\n";
                if (!dbd->getMore()) nbr.peerDbdDone = true;
                sendDBD(from, /*init=*/false, nbr.amMaster, nbr.ddSeq);
                nbr.ddSeq++; // slave increments after response
                tryGoFull(nbr);
            } else if (dbd->getMaster() && dbd->getSeq_no() + 1 == nbr.ddSeq && nbr.lastSentDbd) {
                // Slave repeats its last DBD on master's retransmit.
                MLOG_N_DEBUG(from) << "duplicate DBD from master seq=" << dbd->getSeq_no()
                                   << ", repeating last response\n";
                auto np = std::make_unique<NodePkt>("Myriad_DBD_Pkt");
                np->setType(MYRIADLS_PACKET_TYPE);
                np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
                np->encapsulate(nbr.lastSentDbd->dup());
                countSend("dbd", np->getByteLength());
                sendToNode(from, std::move(np));
            } else {
                // TODO: Generate SeqNumberMismatch: clear DBD/LSR
                // state and restart from ExStart. Currently we just drop the packet.
                MLOG_N_WARN(from) << "received unexpected DBD from master\n";
            }
        }
    }
}

void MyriadLS::sendDBD(nodeid_t to, bool init, bool master, uint16_t seq_no) {
    auto* dbd = new MyriadDBD("Myriad_DBD");
    dbd->setRouter_id_src(myId);
    dbd->setInit(init);
    dbd->setMaster(master);
    dbd->setSeq_no(seq_no);

    // Init DBD carries no LSA summaries (negotiation only). Phase-2 DBDs pick
    // up to dbExchangeBatchSize headers off the per-neighbor queue.
    bool more = true;
    if (!init) {
        auto it = neighbors.find(to);
        if (it != neighbors.end()) {
            auto& q = it->second.dbdQueue;
            size_t batch = std::min(q.size(), static_cast<size_t>(dbExchangeBatchSize));
            dbd->setLs_dbdsArraySize(batch);
            for (size_t i = 0; i < batch; ++i) {
                auto lit = lsdb.find(q.front());
                if (lit != lsdb.end()) dbd->setLs_dbds(i, *lit->second); // slicing copy
                q.pop_front();
            }
            more = !q.empty();
        }
    }
    dbd->setMore(more);
    dbd->setByteLength(dbdBytes(dbd));

    // MS=1 is the side that retransmits.
    // MS=0 (slave response) is fire and forget
    if (master) {
        auto it = neighbors.find(to);
        if (it != neighbors.end()) {
            cancelReliable(it->second.inflightDbd);
            it->second.inflightDbd = sendReliable(to, dbd, rxmtInterval);
            return;
        }
    }
    // Slave path: cache so we can repeat on a duplicate DBD from master.
    auto sit = neighbors.find(to);
    if (sit != neighbors.end()) {
        sit->second.lastSentDbd.reset(dbd->dup());
    }
    auto np = std::make_unique<NodePkt>("Myriad_DBD_Pkt");
    np->setType(MYRIADLS_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(dbd);
    countSend("dbd", np->getByteLength());
    sendToNode(to, std::move(np));
}

} // namespace routingsim
