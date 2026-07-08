#include "routing/MYRIAD/MyriadGp.h"

namespace routingsim {

Define_Module(MyriadGP);

void MyriadGP::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADGP_PACKET_TYPE;

    node().bind(MYRIADGP_PACKET_TYPE, this);

    mySubnets = { (static_cast<uint32_t>(myId) << 8) | 0x01,
                  (static_cast<uint32_t>(myId) << 8) | 0x02 };

    MLOG_INFO() << "MyriadGP up\n";

    // Periodically send hellos to all neighbors. The hello / dead / two-way
    scheduleHello(helloInterval);
}

void MyriadGP::onTwoWayReached(GpNeighbor& nbr) {
    nbr.state = NSTATE_FULL;

    // For each ring we share with this neighbor: originate a collect iff this
    // TWOWAY brings the count of TWOWAY-or-better ring-neighbors on R to
    // exactly 2. The just-transitioned nbr counts.
    for (uint32_t R : nbr.sharedRings) {
        size_t count = 0;
        for (auto& [nid, n] : neighbors) {
            if (n.state < NSTATE_TWOWAY) continue;
            for (uint32_t r : n.sharedRings) if (r == R) { ++count; break; }
        }
        if (count == 2 && rings[R].inflight.tx == nullptr) {
            originateCollect(R, nbr.id);
        } else if (count > 2) {
            MLOG_N_WARN(nbr.id) << "ring '" << (char)R
                << "' has >2 ring-neighbors (misconfig); ignoring\n";
        }
    }
}

void MyriadGP::originateCollect(uint32_t ring, nodeid_t to) {
    RingState& rs = rings[ring];
    rs.inflight.seq_no += 1;

    auto* pkt = new MyriadGpCollect("Myriad_GP_Collect");
    pkt->setType(MYRIADGP_COLLECT);
    pkt->setRouter_id_src(myId);     // king_id
    pkt->setSeq_no(rs.inflight.seq_no);
    pkt->setCurrent_ring(ring);
    pkt->setRandom_no(0);            // TODO: RN generation when dedup lands

    // Insert my (subnet, myId) pairs.
    pkt->setRrArraySize(mySubnets.size());
    for (size_t i = 0; i < mySubnets.size(); ++i) {
        MyriadGpRr e; e.subnet = mySubnets[i]; e.owner = myId;
        pkt->setRr(i, e);
    }

    // Seed transitions with my OTHER rings, all isRedundant=false; the ring
    // fills them in as members insert their own memberships.
    std::vector<uint32_t> others;
    for (uint32_t r : selfRings) if (r != ring) others.push_back(r);
    pkt->setTransitionsArraySize(others.size());
    for (size_t i = 0; i < others.size(); ++i) {
        MyriadGpRingTransition t; t.ring = others[i]; t.isRedundant = false;
        pkt->setTransitions(i, t);
    }

    rs.inflight.tx = sendReliable(to, pkt, rxmtInterval);
    // Parallel budget: (retryLimit + 0.5) * rxmtInterval — fires halfway between
    // the last retransmit and what would have been the next one, so a return
    // that came back in time has already cancelled us.
    rs.inflight.budgetTimer = addOneShotTimer(
        (COLLECT_RETRY_LIMIT + 0.5) * rxmtInterval,
        [this, ring](short) {
            onCollectBudgetExpired(ring);
            return false;
        });

    MLOG_N_INFO(to) << "originate COLLECT ring='" << (char)ring
        << "' seq=" << rs.inflight.seq_no << "\n";
}

void MyriadGP::cancelInflight(uint32_t ring) {
    auto it = rings.find(ring);
    if (it == rings.end()) return;
    RingInflight& fl = it->second.inflight;
    if (fl.tx) cancelReliable(fl.tx);
    if (fl.budgetTimer) {
        cancelAndDelete(fl.budgetTimer);
        fl.budgetTimer = nullptr;
    }
}

void MyriadGP::onCollectBudgetExpired(uint32_t ring) {
    RingState& rs = rings[ring];
    rs.inflight.budgetTimer = nullptr;
    if (rs.inflight.tx == nullptr) return; // already resolved
    cancelReliable(rs.inflight.tx);
    MLOG_WARN() << "COLLECT budget expired ring='" << (char)ring
                << "' seq=" << rs.inflight.seq_no << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onNeighborDead(GpNeighbor& nbr, bool /*wasFull*/) {
    // For each ring shared with the dead neighbor: if no inflight for that
    // ring, originate a probe COLLECT to the dead neighbor. Its retransmits
    // act as the liveness grace period — if the neighbor recovers within the
    // budget, the collect completes; otherwise the budget timer fires DEGRADE.
    // If an inflight already exists, do nothing and let its budget run.
    for (uint32_t R : nbr.sharedRings) {
        if (rings[R].inflight.tx != nullptr) continue;
        MLOG_N_INFO(nbr.id) << "grace COLLECT ring='" << (char)R
                            << "' to dead neighbor\n";
        originateCollect(R, nbr.id);
    }
}

void MyriadGP::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    nodeid_t sender = pkt->getHop().from;  // immediate hop, not the king
    auto inner = std::unique_ptr<cPacket>(pkt->decapsulate());

    // Hello packets come from MyriadHello so dont go through gpheader path
    if (auto* hello = dynamic_cast<MyriadHello*>(inner.get())) {
        handleHello(hello, hello->getRouter_id_src());
        return;
    }

    auto* header = check_and_cast<MyriadGpHeader*>(inner.get());
    switch (header->getType()) {
        case MYRIADGP_COLLECT:
            handleCollect(check_and_cast<MyriadGpCollect*>(header), sender);
            break;
        case MYRIADGP_UPDATE:
            handleUpdate(check_and_cast<MyriadGpUpdate*>(header), sender);
            break;
        case MYRIADGP_DEGRADE:
            handleDegrade(check_and_cast<MyriadGpDegrade*>(header), sender);
            break;
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }
}

void MyriadGP::handleCollect(MyriadGpCollect* collect, nodeid_t sender) {
    uint32_t R = collect->getCurrent_ring();
    nodeid_t king = collect->getRouter_id_src();
    uint32_t seq = collect->getSeq_no();
    RingState& rs = rings[R];

    // Own return — circular ACK completes the collection.
    if (king == myId) {
        if (rs.inflight.tx != nullptr && seq == rs.inflight.seq_no) {
            cancelInflight(R);

            // Snapshot the collected data onto ring state.
            rs.rr.clear();
            rs.rr.reserve(collect->getRrArraySize());
            for (size_t i = 0; i < collect->getRrArraySize(); ++i)
                rs.rr.push_back(collect->getRr(i));

            rs.adjacentRings.clear();
            rs.adjacentRings.reserve(collect->getTransitionsArraySize());
            for (size_t i = 0; i < collect->getTransitionsArraySize(); ++i) {
                auto& t = collect->getTransitions(i);
                rs.adjacentRings.push_back({ t.ring, t.isRedundant });
            }

            MLOG_INFO() << "COLLECT complete ring='" << (char)R
                        << "' seq=" << seq
                        << " rr=" << rs.rr.size()
                        << " transitions=[";
            for (size_t i = 0; i < rs.adjacentRings.size(); ++i) {
                auto& a = rs.adjacentRings[i];
                EV << (i ? "," : "") << (char)a.ring << (a.isRedundant ? "=D" : "=S");
            }
            EV << "]\n";

            // TODO: originate UPDATE for R (rr map, chain=[R], spof_count=0;
            //       piggyback widths trailer if adjacentRings changed).
        } else {
            MLOG_N_WARN(sender) << "stale own COLLECT return ring='" << (char)R
                                << "' seq=" << seq << " (cur=" << rs.inflight.seq_no << ")\n";
        }
        return;
    }

    // Contention: I have an inflight for this ring → someone else is also King.
    if (rs.inflight.tx != nullptr) {
        if (king > myId) {
            // Yield: cancel my transmit and become a plain forwarder.
            MLOG_N_INFO(sender) << "yield COLLECT ring='" << (char)R
                                << "' to king=" << (int)king << "\n";
            cancelInflight(R);
            insertAndForwardCollect(collect, R, sender);
        } else {
            // Ignore lower-id king; my collection continues.
            MLOG_N_DEBUG(sender) << "ignore COLLECT ring='" << (char)R
                                 << "' from lower king=" << (int)king << "\n";
        }
        return;
    }

    // Plain forwarder: insert and pass on.
    insertAndForwardCollect(collect, R, sender);
}

nodeid_t MyriadGP::otherRingNeighbor(uint32_t ring, nodeid_t sender) {
    for (auto& [nid, n] : neighbors) {
        if (nid == sender) continue;
        if (n.state < NSTATE_TWOWAY) continue;
        for (uint32_t r : n.sharedRings) if (r == ring) return nid;
    }
    return undefined_node;
}

void MyriadGP::insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t R, nodeid_t sender) {
    // 1. Append my (subnet, myId) pairs.
    size_t n = pkt->getRrArraySize();
    pkt->setRrArraySize(n + mySubnets.size());
    for (size_t i = 0; i < mySubnets.size(); ++i) {
        MyriadGpRr e; e.subnet = mySubnets[i]; e.owner = myId;
        pkt->setRr(n + i, e);
    }

    // 2. Update transitions for each of my OTHER rings.
    for (uint32_t R2 : selfRings) {
        if (R2 == R) continue;
        bool found = false;
        for (size_t i = 0; i < pkt->getTransitionsArraySize(); ++i) {
            auto& t = pkt->getTransitionsForUpdate(i);
            if (t.ring != R2) continue;
            found = true;
            if (!t.isRedundant) t.isRedundant = true; // second contributor confirms double
            break;
        }
        if (!found) {
            size_t k = pkt->getTransitionsArraySize();
            pkt->setTransitionsArraySize(k + 1);
            MyriadGpRingTransition t; t.ring = R2; t.isRedundant = false;
            pkt->setTransitions(k, t);
        }
    }

    // 3. Forward to the OTHER ring-neighbor for R.
    nodeid_t to = otherRingNeighbor(R, sender);
    if (to == undefined_node) {
        MLOG_N_WARN(sender) << "no other ring-neighbor for ring='" << (char)R
                            << "'; dropping COLLECT\n";
        return;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Collect_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->encapsulate(pkt->dup());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward COLLECT ring='" << (char)R
                     << "' seq=" << pkt->getSeq_no()
                     << " king=" << (int)pkt->getRouter_id_src() << "\n";
}

void MyriadGP::handleUpdate(MyriadGpUpdate* /*update*/, nodeid_t from) {
    // TODO: install / dedup / forward path-vector update.
    MLOG_N_DEBUG(from) << "Received Update (handler stub)\n";
}

void MyriadGP::handleDegrade(MyriadGpDegrade* /*degrade*/, nodeid_t from) {
    // TODO: process ring-degrade event.
    MLOG_N_DEBUG(from) << "Received Degrade (handler stub)\n";
}

void MyriadGP::reset() {
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.deadTimer) {
            cancelAndDelete(nbr.deadTimer);
            nbr.deadTimer = nullptr;
        }
    }
    neighbors.clear();
    for (auto& [rid, rs] : rings) {
        if (rs.inflight.tx) cancelReliable(rs.inflight.tx);
        if (rs.inflight.budgetTimer) {
            cancelAndDelete(rs.inflight.budgetTimer);
            rs.inflight.budgetTimer = nullptr;
        }
    }
    rings.clear();
    cancelHelloTimer();
}

void MyriadGP::finish() {
    // TODO: dump path-vector table.
}

} // namespace routingsim
