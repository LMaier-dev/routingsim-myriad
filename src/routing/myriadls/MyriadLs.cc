#include "routing/myriadls/MyriadLs.h"
#include <algorithm>

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    MyriadHelper::onInitialize();
    packetType = MYRIADLS_PACKET_TYPE;
    helloInterval = par("helloInterval");
    deadInterval = par("deadInterval");
    rxmtInterval = par("rxmtInterval");
    dbExchangeBatchSize = par("dbExchangeBatchSize");

    node().bind(MYRIADLS_PACKET_TYPE, this);

    switch (myId)
    {
    case 0:
        rings = "A";
        break;
    case 1:
        rings = "A,B";
        break;
    case 2:
        rings = "B,C";
        break;
    case 3:
        rings = "A,B";
        break;
    case 4:
        rings = "B";
        break;
    case 5:
        rings = "C";
        break;
    case 6:
        rings = "B,C";
        break;
    default:
        break;
    }

    MLOG_INFO() << "rings: " << rings << "\n";

    installSelfLsas();

    // periodically send hellos to all neighbors
    scheduleHello(helloInterval);

}

NeighborState MyriadLS::getNeighborState(nodeid_t nbr) const {
    auto it = neighbors.find(nbr);
    if (it == neighbors.end()) return NSTATE_DOWN;
    return it->second.state;
}

void MyriadLS::onConnect(nodeid_t nid) {
    // Retrieve OR create neighbor entry
    NeighborEntry& nbr = neighbors[nid];
    nbr.id = nid;
    nbr.state = NSTATE_DOWN;
}

void MyriadLS::onDisconnect(nodeid_t nid) {
    handleDead(nid);
}

void MyriadLS::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    auto header = std::unique_ptr<MyriadHeader>(check_and_cast<MyriadHeader*>(pkt->decapsulate()));
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
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }

}

void MyriadLS::reset() {
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.deadTimer) {
            cancelAndDelete(nbr.deadTimer);
            nbr.deadTimer = nullptr;
        }
        cancelReliable(nbr.pendingDbd);
        cancelReliable(nbr.pendingLsr);
        nbr.lastSentDbd.reset();
    }
    neighbors.clear();
    lsdb.clear();
    installSelfLsas();

    if (helloTimer) {
        cancelAndDelete(helloTimer);
        helloTimer = nullptr;
    }
}


void MyriadLS::installSelfLsas() {
    // Rings LSA: ring string char code as a uint64 entry
    auto ringsLsa = std::make_unique<RingsLsa>();
    ringsLsa->setSeq_no(1);
    ringsLsa->setLsa_origin(myId);

    std::vector<uint64_t> ringIds;
    std::stringstream ss(rings);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) ringIds.push_back(static_cast<uint64_t>(tok[0]));
    }
    ringsLsa->setRingsArraySize(ringIds.size());
    for (size_t i = 0; i < ringIds.size(); ++i)
        ringsLsa->setRings(i, ringIds[i]);

    lsdb[LsaKey{myId, LSA_RINGS}] = std::move(ringsLsa);

    // Subnets LSA: bogus subnets derived from node id
    auto subnetsLsa = std::make_unique<SubnetsLsa>();
    subnetsLsa->setSeq_no(1);
    subnetsLsa->setLsa_origin(myId);
    subnetsLsa->setSubnetsArraySize(2);
    subnetsLsa->setSubnets(0, (static_cast<uint64_t>(myId) << 8) | 0x01);
    subnetsLsa->setSubnets(1, (static_cast<uint64_t>(myId) << 8) | 0x02);

    lsdb[LsaKey{myId, LSA_SUBNETS}] = std::move(subnetsLsa);
    
    originateAdjacenciesLsa();
}

void MyriadLS::originateAdjacenciesLsa() {
    std::vector<nodeid_t> adj;
    for (const auto& [nid, nbr] : neighbors) {
        if (nbr.state == NSTATE_FULL) adj.push_back(nid);
    }

    LsaKey key{myId, LSA_ADJACENCIES};
    uint16_t newSeq = 1;
    auto it = lsdb.find(key);
    if (it != lsdb.end()) newSeq = it->second->getSeq_no() + 1;

    auto lsa = std::make_unique<AdjacenciesLsa>();
    lsa->setLsa_origin(myId);
    lsa->setSeq_no(newSeq);
    lsa->setAdjacenciesArraySize(adj.size());
    for (size_t i = 0; i < adj.size(); ++i) lsa->setAdjacencies(i, adj[i]);

    lsdb[key] = std::move(lsa);
    MLOG_INFO() << "originated AdjacenciesLsa seq=" << newSeq
                << " with " << adj.size() << " adjacent neighbor(s)\n";
    // TODO: flood this LSA to every FULL neighbor via a sendReliable LSU
}

void MyriadLS::buildPendingDbd(NeighborEntry& nbr) {
    nbr.dbdPending.clear();
    for (const auto& kv : lsdb) {
        nbr.dbdPending.push_back(kv.first);
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
                           << " seq=" << h.getSeq_no()
                           << " age=" << h.getAge() << "\n";
    }
}

void MyriadLS::logLsdb() {
    if (!logEnabled(MLOG_LEVEL_INFO)) return;
    MLOG_INFO() << "LSDB (" << lsdb.size() << " entries):\n";
    for (const auto& [key, lsa] : lsdb) {
        MLOG_INFO() << "  origin=" << (int)key.origin
                    << " type=" << key.type
                    << " seq=" << lsa->getSeq_no()
                    << " age=" << lsa->getAge();
        switch (lsa->getType()) {
            case LSA_RINGS: {
                auto* r = check_and_cast<RingsLsa*>(lsa.get());
                EV << " rings=[";
                for (size_t i = 0; i < r->getRingsArraySize(); ++i)
                    EV << (i ? "," : "") << (char)r->getRings(i);
                EV << "]";
                break;
            }
            case LSA_SUBNETS: {
                auto* s = check_and_cast<SubnetsLsa*>(lsa.get());
                EV << " subnets=[";
                for (size_t i = 0; i < s->getSubnetsArraySize(); ++i)
                    EV << (i ? "," : "") << s->getSubnets(i);
                EV << "]";
                break;
            }
            case LSA_ADJACENCIES: {
                auto* a = check_and_cast<AdjacenciesLsa*>(lsa.get());
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

void MyriadLS::scheduleHello(simtime_t delay) {
    helloTimer = addOneShotTimer(delay, [this](short) {
        helloTimer = nullptr; // framework deletes this TimerMsg after we return
        sendHello();          // reschedules and assigns the new helloTimer
        return false;
    });
}

void MyriadLS::sendHello() {
    auto hello = std::make_unique<MyriadHello>("Myriad_Hello");
    hello->setRouter_id_src(myId);
    // Currently not read on the receiving end, but set it anyway for completeness
    hello->setHelloInterval(helloInterval);
    hello->setDeadInterval(deadInterval);

    // Only advertise neighbors we've actually heard a hello from
    std::vector<nodeid_t> heard;
    heard.reserve(neighbors.size());
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.state >= NSTATE_INIT) heard.push_back(nid);
    }
    hello->setKnownNeighborsArraySize(heard.size());
    for (size_t i = 0; i < heard.size(); ++i) {
        hello->setKnownNeighbors(i, heard[i]);
    }

    // TODO set hello->setByteLength
    for (const auto& [nid, nbr] : neighbors) {
        auto np = std::make_unique<NodePkt>("Myriad_Hello_Pkt");
        np->setType(MYRIADLS_PACKET_TYPE);
        np->encapsulate(hello->dup());
        sendToNode(nid, std::move(np));
    }
    scheduleHello(helloInterval);
}

void MyriadLS::handleHello(MyriadHello* hello, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    resetDead(from);

    // HelloReceived: DOWN -> INIT (can subsequently go into TWOWAY if my id is in their hello)
    if (nbr.state == NSTATE_DOWN) {
        nbr.id = from;
        nbr.state = NSTATE_INIT;
        MLOG_N_INFO(from) << "received first hello (INIT)\n";
    }

    if (nbr.state >= NSTATE_TWOWAY) return; // already past two-way

    // 2-WayReceived: INIT -> TWOWAY
    int size = hello->getKnownNeighborsArraySize();
    for (size_t i = 0; i < size; i++) {
        if (hello->getKnownNeighbors(i) == myId) {
            nbr.state = NSTATE_TWOWAY;
            MLOG_N_INFO(from) << "now in TWO-WAY state\n";
            nbr.ddSeq = 0; // arbitrary starting seq no based on node ids
            nbr.amMaster = true;
            buildPendingDbd(nbr);
            sendDBD(from, /*init=*/true, nbr.amMaster, nbr.ddSeq);
            break;
        }
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
        if (std::find(nbr.lsrPending.begin(), nbr.lsrPending.end(), key) != nbr.lsrPending.end()) continue;
        MLOG_N_DEBUG(nbr.id) << "LSDB missing or stale for "
                << (int)key.origin << "/" << key.type << ", queued for LSR\n";
        nbr.lsrPending.push_back(key);
    }
    // RFC §10.9: requests can be sent concurrently with the DBD exchange.
    // sendNextLsr is a no-op if there's already an LSR in flight; the new keys
    // will go out in the next LSR after the current one is acked by an LSU.
    sendNextLsr(nbr);
}

void MyriadLS::sendNextLsr(NeighborEntry& nbr) {
    if (nbr.lsrPending.empty()) {
        // Drained: this is also the natural close-out point after an LSU
        // satisfied the last outstanding requests.
        tryGoFull(nbr);
        return;
    }
    // RFC §10.9: at most one outstanding LSR per neighbor.
    if (nbr.pendingLsr) return;

    auto* lsr = new MyriadLSR("Myriad_LSR");
    lsr->setRouter_id_src(myId);

    size_t batch = std::min(nbr.lsrPending.size(), static_cast<size_t>(dbExchangeBatchSize));
    lsr->setReqsArraySize(batch);
    auto it = nbr.lsrPending.begin();
    for (size_t i = 0; i < batch; ++i, ++it) {
        LsaRequest r;
        r.origin = it->origin;
        r.type = it->type;
        lsr->setReqs(i, r);
    }
    MLOG_N_DEBUG(nbr.id) << "sending LSR with " << batch << " request(s)\n";
    nbr.pendingLsr = sendReliable(nbr.id, lsr, rxmtInterval);
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
    np->encapsulate(lsu);
    sendToNode(from, std::move(np));
}

void MyriadLS::handleLSU(MyriadLSU* lsu, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    size_t n = lsu->getLsasArraySize();
    MLOG_N_DEBUG(from) << "received LSU with " << n << " LSA(s)\n";

    for (size_t i = 0; i < n; ++i) {
        const LsaHeader* h = lsu->getLsas(i);
        LsaKey key{h->getLsa_origin(), h->getType()};
        // Install if missing or strictly newer; ties leave the LSDB unchanged.
        auto it = lsdb.find(key);
        if (it == lsdb.end() || it->second->getSeq_no() < h->getSeq_no()) {
            lsdb[key] = std::unique_ptr<LsaHeader>(check_and_cast<LsaHeader*>(h->dup()));
            MLOG_N_DEBUG(from) << "installed LSA " << (int)key.origin
                               << "/" << key.type << " seq=" << h->getSeq_no() << "\n";
        }
        // Drain matched key from the request list — independent of install
        // decision (a stale-but-acceptable LSA still satisfies the request).
        auto pit = std::find(nbr.lsrPending.begin(), nbr.lsrPending.end(), key);
        if (pit != nbr.lsrPending.end()) nbr.lsrPending.erase(pit);
    }
    // TODO: distinguish solicited from flooded LSU.
    // Currently every LSU is treated as the response to our outstanding LSR
    // TODO: cancel pendingLsr only if at least one LSA matched lsrPending and forward to the flooding-ack path otherwise.
    cancelReliable(nbr.pendingLsr);
    sendNextLsr(nbr); // sends next batch, or tryGoFull if drained
}

void MyriadLS::tryGoFull(NeighborEntry& nbr) {
    if (nbr.state == NSTATE_FULL) return;
    if (!nbr.dbdPending.empty()) return;
    if (nbr.pendingDbd != nullptr) return;
    if (!nbr.peerDbdDone) return;
    if (!nbr.lsrPending.empty()) return;
    if (nbr.pendingLsr != nullptr) return;
    nbr.state = NSTATE_FULL;
    MLOG_N_INFO(nbr.id) << "going FULL\n";
    // TODO: In handleDead remove the dead neighbor and flood to other FULL neighbors.
    originateAdjacenciesLsa();
}

void MyriadLS::finish() {
    logLsdb();
}

void MyriadLS::handleDBD(MyriadDBD* dbd, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    logDbd(dbd, from);
    if (myId > from) { // master
        if (dbd->getInit()) { // Phase 1 master negotioation
            nbr.amMaster = true;
            nbr.ddSeq = nbr.ddSeq + 1;
            // Phase 2 intiated by master
            cancelReliable(nbr.pendingDbd);
            sendDBD(from, /*init=*/false, nbr.amMaster, nbr.ddSeq);
        } else {
            if (!dbd->getMaster() && dbd->getSeq_no() == nbr.ddSeq && nbr.state < NSTATE_FULL) {
                processDBD(dbd, nbr); // expected DBD, process and respond with next one
                cancelReliable(nbr.pendingDbd);
                MLOG_N_DEBUG(from) << "received expected DBD from slave, seq=" << dbd->getSeq_no() << "\n";
                if (!dbd->getMore()) nbr.peerDbdDone = true;
                nbr.ddSeq++; // master increments before response
                // If both sides are out of LSAs, skip the empty follow-up DBD.
                if (!(nbr.peerDbdDone && nbr.dbdPending.empty())) {
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
            // slave does not retransmit
            cancelReliable(nbr.pendingDbd);
        } else {
            // slaves never retransmit (cancel just in case, but it should be null)
            cancelReliable(nbr.pendingDbd);
            // Phase 2 response by slave
            if (dbd->getMaster() && dbd->getSeq_no() == nbr.ddSeq && nbr.state < NSTATE_FULL) {
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
                np->encapsulate(nbr.lastSentDbd->dup());
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

    // Init DBD carries no LSA summaries (negotiation only). Phase-2 DBDs drain
    // up to dbExchangeBatchSize headers off the per-neighbor queue; `more` is
    // true iff anything still remains after this batch.
    bool more = true;
    if (!init) {
        auto it = neighbors.find(to);
        if (it != neighbors.end()) {
            auto& q = it->second.dbdPending;
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

    // MS=1 is the side that retransmits.
    // MS=0 (slave response) is fire and forget
    if (master) {
        auto it = neighbors.find(to);
        if (it != neighbors.end()) {
            cancelReliable(it->second.pendingDbd);
            it->second.pendingDbd = sendReliable(to, dbd, rxmtInterval);
            return;
        }
    }
    // Slave path: cache so we can repeat on a duplicate DBD from master (§10.6).
    auto sit = neighbors.find(to);
    if (sit != neighbors.end()) {
        sit->second.lastSentDbd.reset(dbd->dup());
    }
    auto np = std::make_unique<NodePkt>("Myriad_DBD_Pkt");
    np->setType(MYRIADLS_PACKET_TYPE);
    np->encapsulate(dbd);
    sendToNode(to, std::move(np));
}

void MyriadLS::resetDead(nodeid_t nid) {
    auto it = neighbors.find(nid);
    if (it == neighbors.end()) return;
    NeighborEntry& nbr = it->second;
    // nullptr is accepted (i.e. on first hello)
    cancelAndDelete(nbr.deadTimer);
    nbr.deadTimer = addOneShotTimer(deadInterval, [this, nid](short) {
        handleDead(nid);
        return false;
    });
}

void MyriadLS::handleDead(nodeid_t nid) {
    auto it = neighbors.find(nid);
    if (it == neighbors.end()) return;
    NeighborEntry& nbr = it->second;

    MLOG_N_WARN(nid) << "neighbor timed out (DEAD)\n";
    nbr.deadTimer = nullptr;
    nbr.state = NSTATE_DOWN;
    cancelReliable(nbr.pendingDbd);
    cancelReliable(nbr.pendingLsr);
    nbr.amMaster = false;
    nbr.ddSeq = 0;
    nbr.dbdPending.clear();
    nbr.lsrPending.clear();
    nbr.peerDbdDone = false;
    nbr.lastSentDbd.reset();
}

} // namespace routingsim

