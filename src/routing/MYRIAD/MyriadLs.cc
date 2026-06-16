#include "routing/MYRIAD/MyriadLs.h"
#include <algorithm>

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADLS_PACKET_TYPE;
    dbExchangeBatchSize = par("dbExchangeBatchSize");

    node().bind(MYRIADLS_PACKET_TYPE, this);


    installSelfLsas();

    // periodically send hellos to all neighbors
    scheduleHello(helloInterval);

}

void MyriadLS::onTwoWayReached(NeighborEntry& nbr) {
    nbr.ddSeq = 0; // arbitrary starting seq no based on node ids
    nbr.amMaster = true;
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

void MyriadLS::reset() {
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
    installSelfLsas();

    cancelHelloTimer();
}


void MyriadLS::buildSelfRingsLsa(uint16_t seq) {
    auto lsa = std::make_unique<RingsLsa>();
    lsa->setSeq_no(seq);
    lsa->setLsa_origin(myId);
    lsa->setRingsArraySize(selfRings.size());
    for (size_t i = 0; i < selfRings.size(); ++i) lsa->setRings(i, selfRings[i]);

    lsdb[LsaKey{myId, LSA_RINGS}] = std::move(lsa);
}

void MyriadLS::buildSelfSubnetsLsa(uint16_t seq) {
    auto lsa = std::make_unique<SubnetsLsa>();
    lsa->setSeq_no(seq);
    lsa->setLsa_origin(myId);
    // Bogus subnets derived from node id: <id>.1 and <id>.2
    lsa->setSubnetsArraySize(2);
    lsa->setSubnets(0, (static_cast<uint64_t>(myId) << 8) | 0x01);
    lsa->setSubnets(1, (static_cast<uint64_t>(myId) << 8) | 0x02);

    lsdb[LsaKey{myId, LSA_SUBNETS}] = std::move(lsa);
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

    lsdb[LsaKey{myId, LSA_ADJACENCIES}] = std::move(lsa);
}

void MyriadLS::installSelfLsas() {
    originateSelfLsa(LSA_RINGS);
    originateSelfLsa(LSA_SUBNETS);
    originateSelfLsa(LSA_ADJACENCIES);
}

void MyriadLS::originateSelfLsa(MyriadLSAType type, uint16_t minSeq) {
    LsaKey key{myId, type};
    auto it = lsdb.find(key);
    uint16_t newSeq = (it != lsdb.end()) ? it->second->getSeq_no() + 1 : 1;
    if (static_cast<uint16_t>(minSeq + 1) > newSeq) newSeq = minSeq + 1;

    switch (type) {
        case LSA_RINGS:       buildSelfRingsLsa(newSeq);       break;
        case LSA_SUBNETS:     buildSelfSubnetsLsa(newSeq);     break;
        case LSA_ADJACENCIES: buildSelfAdjacenciesLsa(newSeq); break;
    }

    MLOG_INFO() << "originated self-LSA type=" << type << " seq=" << newSeq << "\n";

    for (auto& [nid, nbr] : neighbors) {
        if (nbr.state != NSTATE_FULL) continue;
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
    np->encapsulate(lsu);
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
                lsdb[key] = std::unique_ptr<LsaHeader>(check_and_cast<LsaHeader*>(h->dup()));
                installed = true;
                MLOG_N_DEBUG(from) << "installed LSA " << (int)key.origin
                                   << "/" << key.type << " seq=" << h->getSeq_no() << "\n";
            }
        }

        // LSR response vs flooded LSU
        auto pit = std::find(nbr.lsrQueue.begin(), nbr.lsrQueue.end(), key);
        if (pit != nbr.lsrQueue.end()) {
            nbr.lsrQueue.erase(pit);
            drainedAny = true;
        } else {
            // Flooded: ack to sender, and reflood if we just installed newer.
            toAck.push_back(h);
            if (installed) toReflood.push_back(key);
        }
    }

    // Reflood all newly-installed keys to FULL neighbors except sender.
    // They get bundled into one LSU per neighbor (per-queue)
    if (!toReflood.empty()) {
        for (auto& [nid, nb] : neighbors) {
            if (nb.state != NSTATE_FULL) continue;
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
    np->encapsulate(ack);
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
            cancelReliable(nbr.inflightDbd);
            sendDBD(from, /*init=*/false, nbr.amMaster, nbr.ddSeq);
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
            // slave does not retransmit
            cancelReliable(nbr.inflightDbd);
        } else {
            // slaves never retransmit (cancel just in case, but it should be null)
            cancelReliable(nbr.inflightDbd);
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
    np->encapsulate(dbd);
    sendToNode(to, std::move(np));
}

} // namespace routingsim
