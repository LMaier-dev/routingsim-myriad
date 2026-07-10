#include "routing/MYRIAD/MyriadGp.h"
#include <algorithm>

namespace routingsim {

Define_Module(MyriadGP);

void MyriadGP::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADGP_PACKET_TYPE;

    node().bind(MYRIADGP_PACKET_TYPE, this);

    mySubnets = { (static_cast<uint32_t>(myId) << 8) | 0x01,
                  (static_cast<uint32_t>(myId) << 8) | 0x02 };

    refreshInterval = par("refreshInterval");
    refreshJitter = par("refreshJitter");

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
        if (count > rings[R].maxRingNeighbors) rings[R].maxRingNeighbors = count;
        if (count >= 1 && count <= 2 && rings[R].collectInflight.tx == nullptr) {
            originateCollect(R, nbr.id);
        } else if (count > 2) {
            MLOG_N_WARN(nbr.id) << "ring '" << (char)R
                << "' has >2 ring-neighbors (misconfig); ignoring\n";
        }
    }
}

void MyriadGP::originateCollect(uint32_t ring, nodeid_t to) {
    RingState& currentRingState = rings[ring];
    currentRingState.originSeq += 1;
    currentRingState.collectInflight.seq_no = currentRingState.originSeq;

    auto* pkt = new MyriadGpCollect("Myriad_GP_Collect");
    pkt->setType(MYRIADGP_COLLECT);
    pkt->setRouter_id_src(myId);     // king_id
    pkt->setSeq_no(currentRingState.originSeq);
    pkt->setCurrent_ring(ring);

    // Insert my (subnet, myId) pairs.
    pkt->setRrArraySize(mySubnets.size());
    for (size_t i = 0; i < mySubnets.size(); ++i) {
        MyriadGpRr rr_entry; rr_entry.subnet = mySubnets[i];
        rr_entry.owner = myId;
        pkt->setRr(i, rr_entry);
    }

    // Seed transitions with my OTHER rings, all isRedundant=false; the ring
    // fills them in as members insert their own memberships.
    std::vector<uint32_t> transitions;
    for (uint32_t r : selfRings) if (r != ring) transitions.push_back(r);
    pkt->setTransitionsArraySize(transitions.size());
    for (size_t i = 0; i < transitions.size(); ++i) {
        MyriadGpRingTransition t; t.ring = transitions[i]; t.isRedundant = false;
        pkt->setTransitions(i, t);
    }

    // Send one direction now; retransmits alternate to the other neighbor so a
    // single directional send still reaches every ring member if one side breaks.
    sendRingReliable(currentRingState.collectInflight, to, to, pkt, rxmtInterval,
                     COLLECT_RETRY_LIMIT, [this, ring]() { onCollectBudgetExpired(ring); });

    MLOG_N_INFO(to) << "originate COLLECT ring='" << (char)ring
        << "' seq=" << currentRingState.originSeq << "\n";
}

void MyriadGP::reoriginateCollect(uint32_t ring) {
    // Pick either ring-neighbor as the send target (alternation covers the
    // other direction on retransmit); skip if the ring isn't fully seen.
    std::vector<nodeid_t> nbrs = ringNeighbors(ring);
    if (nbrs.empty()) {
        MLOG_DEBUG() << "re-collect ring='" << (char)ring
                     << "' skipped: no ring-neighbors\n";
        return;
    }
    if (rings[ring].collectInflight.tx != nullptr) return; // one at a time
    originateCollect(ring, nbrs[0]);
}

void MyriadGP::startRefresh(uint32_t ring) {
    RingState& rs = rings[ring];
    if (rs.watchdogTimer) { cancelAndDelete(rs.watchdogTimer); rs.watchdogTimer = nullptr; }
    if (rs.refreshTimer)  { cancelAndDelete(rs.refreshTimer);  rs.refreshTimer = nullptr; }
    rs.refreshTimer = addOneShotTimer(refreshInterval, [this, ring](short) {
        rings[ring].refreshTimer = nullptr;
        reoriginateCollect(ring);
        if (rings[ring].isKing) startRefresh(ring); // reschedule
        return false;
    });
}

void MyriadGP::startWatchdog(uint32_t ring) {
    RingState& rs = rings[ring];
    if (rs.watchdogTimer) { cancelAndDelete(rs.watchdogTimer); rs.watchdogTimer = nullptr; }
    simtime_t delay = 2 * refreshInterval + uniform(0, refreshJitter);
    rs.watchdogTimer = addOneShotTimer(delay, [this, ring](short) {
        rings[ring].watchdogTimer = nullptr;
        MLOG_INFO() << "watchdog fired for ring='" << (char)ring
                    << "' — starting coup\n";
        reoriginateCollect(ring); // become king only on own-return
        return false;
    });
}

void MyriadGP::cancelKingship(uint32_t ring) {
    RingState& rs = rings[ring];
    rs.isKing = false;
    if (rs.refreshTimer) { cancelAndDelete(rs.refreshTimer); rs.refreshTimer = nullptr; }
    startWatchdog(ring);
}

void MyriadGP::sendRingReliable(RingInflight& fl, nodeid_t primary, nodeid_t fallback,
                                cPacket* pkt, simtime_t interval, int retryLimit,
                                std::function<void()> onBudgetExpired) {
    auto* h = new RingTx{pkt, primary, fallback, interval, nullptr, false};

    // First send goes to the primary neighbor.
    auto np = std::make_unique<NodePkt>(pkt->getName());
    np->setType(packetType);
    np->encapsulate(pkt->dup());
    sendToNode(primary, std::move(np));

    h->timer = addOneShotTimer(interval, [this, h](short) {
        ringRetxFire(h);
        return false;
    });
    fl.tx = h;

    // Parallel budget: (retryLimit + 0.5) * interval — fires halfway between the
    // last retransmit and what would have been the next one, so a return that
    // came back in time has already cancelled us.
    fl.budgetTimer = addOneShotTimer((retryLimit + 0.5) * interval,
        [onBudgetExpired](short) {
            onBudgetExpired();
            return false;
        });
}

void MyriadGP::ringRetxFire(RingTx* h) {
    h->onFallback = !h->onFallback;
    nodeid_t to = h->onFallback ? h->fallback : h->primary;

    auto np = std::make_unique<NodePkt>(h->pkt->getName());
    np->setType(packetType);
    np->encapsulate(h->pkt->dup());
    sendToNode(to, std::move(np));

    h->timer = addOneShotTimer(h->interval, [this, h](short) {
        ringRetxFire(h);
        return false;
    });
}

void MyriadGP::cancelRingReliable(RingInflight& fl) {
    if (fl.tx) {
        if (fl.tx->timer) cancelAndDelete(fl.tx->timer);
        delete fl.tx->pkt;
        delete fl.tx;
        fl.tx = nullptr;
    }
    if (fl.budgetTimer) {
        cancelAndDelete(fl.budgetTimer);
        fl.budgetTimer = nullptr;
    }
}

void MyriadGP::cancelCollect(uint32_t ring) {
    auto it = rings.find(ring);
    if (it == rings.end()) return;
    cancelRingReliable(it->second.collectInflight);
}

void MyriadGP::onCollectBudgetExpired(uint32_t ring) {
    RingState& currentRingState = rings[ring];
    // This runs from the budgetTimer's own callback; the harness frees that
    // message after we return, so drop our handle before cancelRingReliable
    // (which would otherwise cancelAndDelete the message we're executing).
    currentRingState.collectInflight.budgetTimer = nullptr;
    if (currentRingState.collectInflight.tx == nullptr) return; // already resolved
    cancelRingReliable(currentRingState.collectInflight);
    MLOG_WARN() << "ring send budget expired ring='" << (char)ring
                << "' seq=" << currentRingState.collectInflight.seq_no << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onUpdateBudgetExpired(uint32_t ring, std::vector<uint32_t> chain) {
    auto rit = rings.find(ring);
    if (rit == rings.end()) return;
    auto it = rit->second.updateInflights.find(chain);
    if (it == rit->second.updateInflights.end()) return;
    // Drop the budget-timer handle first (see onCollectBudgetExpired).
    it->second.budgetTimer = nullptr;
    cancelRingReliable(it->second);
    rit->second.updateInflights.erase(it);
    MLOG_WARN() << "UPDATE traversal budget expired ring='" << (char)ring
                << "' chain-len=" << chain.size() << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onNeighborDead(GpNeighbor& nbr, bool /*wasFull*/) {
    // For each ring shared with the dead neighbor: if no inflight for that
    // ring, originate a probe COLLECT to the dead neighbor. Its retransmits
    // act as the liveness grace period — if the neighbor recovers within the
    // budget, the collect completes; otherwise the budget timer fires DEGRADE.
    // If an inflight already exists, do nothing and let its budget run.
    for (uint32_t R : nbr.sharedRings) {
        if (rings[R].collectInflight.tx != nullptr) continue;
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

    // DEGRADE is a standalone class (does not extend MyriadGpHeader).
    if (auto* degrade = dynamic_cast<MyriadGpDegrade*>(inner.get())) {
        handleDegrade(degrade, sender);
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
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }
}

void MyriadGP::handleCollect(MyriadGpCollect* collect, nodeid_t sender) {
    uint32_t R = collect->getCurrent_ring();
    nodeid_t king = collect->getRouter_id_src();
    uint32_t seq = collect->getSeq_no();
    RingState& currentRingState = rings[R];

    // Own return — circular ACK completes the collection.
    if (king == myId) {
        if (currentRingState.collectInflight.tx != nullptr && seq == currentRingState.collectInflight.seq_no) {
            cancelCollect(R);

            // Snapshot the collected data onto ring state.
            currentRingState.rr.clear();
            currentRingState.rr.reserve(collect->getRrArraySize());
            for (size_t i = 0; i < collect->getRrArraySize(); ++i)
                currentRingState.rr.push_back(collect->getRr(i));

            currentRingState.adjacentRings.clear();
            currentRingState.adjacentRings.reserve(collect->getTransitionsArraySize());
            for (size_t i = 0; i < collect->getTransitionsArraySize(); ++i) {
                auto& t = collect->getTransitions(i);
                currentRingState.adjacentRings.push_back({ t.ring, t.isRedundant });
            }

            MLOG_INFO() << "COLLECT complete ring='" << (char)R
                        << "' seq=" << seq
                        << " rr=" << currentRingState.rr.size()
                        << " transitions=[";
            for (size_t i = 0; i < currentRingState.adjacentRings.size(); ++i) {
                auto& a = currentRingState.adjacentRings[i];
                EV << (i ? "," : "") << (char)a.ring << (a.isRedundant ? "=D" : "=S");
            }
            EV << "]\n";

            originateUpdate(R);

            // Winning the lap makes me (or keeps me) king: run the refresh
            // timer, stop watching for someone else.
            if (!currentRingState.isKing) {
                currentRingState.isKing = true;
                MLOG_INFO() << "became king of ring='" << (char)R << "'\n";
            }
            startRefresh(R);
        } else {
            MLOG_N_WARN(sender) << "stale own COLLECT return ring='" << (char)R
                                << "' seq=" << seq << " (cur=" << currentRingState.collectInflight.seq_no << ")\n";
        }
        return;
    }

    // A COLLECT from another node proves that ring's king is alive: reset my
    // watchdog so I don't launch a coup. Kings don't watch (they originate).
    if (!currentRingState.isKing) startWatchdog(R);

    // Contention: I'm also asserting this ring if I have a collect inflight or
    // I'm the established king. Higher id wins; I yield/demote to it, or I hold
    // and ignore a lower id (my next collect makes them yield).
    if (currentRingState.collectInflight.tx != nullptr || currentRingState.isKing) {
        if (king > myId) {
            MLOG_N_INFO(sender) << "yield COLLECT ring='" << (char)R
                                << "' to king=" << (int)king << "\n";
            cancelCollect(R);                     // no-op if no inflight
            if (currentRingState.isKing) cancelKingship(R);
            insertAndForwardCollect(collect, R, sender);
        } else {
            // Ignore lower-id king; my collection/kingship stands.
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

std::vector<nodeid_t> MyriadGP::ringNeighbors(uint32_t ring) {
    std::vector<nodeid_t> out;
    for (auto& [nid, n] : neighbors) {
        if (n.state < NSTATE_TWOWAY) continue;
        for (uint32_t r : n.sharedRings) if (r == ring) { out.push_back(nid); break; }
    }
    return out;
}

void MyriadGP::originateUpdate(uint32_t ring) {
    RingState& rs = rings[ring];

    std::vector<nodeid_t> nbrs = ringNeighbors(ring);
    if (nbrs.empty()) {
        MLOG_WARN() << "originate UPDATE ring='" << (char)ring
                    << "' but no ring-neighbors; skipping\n";
        return;
    }
    // 2-node ring has a single neighbor: probe mode (primary == fallback).
    nodeid_t primary = nbrs[0];
    nodeid_t fallback = nbrs.size() > 1 ? nbrs[1] : nbrs[0];

    auto* pkt = new MyriadGpUpdate("Myriad_GP_Update");
    pkt->setType(MYRIADGP_UPDATE);
    pkt->setRouter_id_src(myId);          // ringking for this ring traversal
    pkt->setSeq_no(rs.originSeq);         // same seq as the collect round
    pkt->setCurrent_ring(ring);
    pkt->setRing_chainArraySize(0);       // origin: empty chain
    pkt->setSpof_count(0);

    pkt->setRrArraySize(rs.rr.size());
    for (size_t i = 0; i < rs.rr.size(); ++i) pkt->setRr(i, rs.rr[i]);

    // Trailer: my (the king's) transition widths for this ring, so members can
    // set spof_count correctly when they transition this update onward.
    pkt->setWidthsArraySize(rs.adjacentRings.size());
    for (size_t i = 0; i < rs.adjacentRings.size(); ++i) {
        MyriadGpRingTransition t;
        t.ring = rs.adjacentRings[i].ring;
        t.isRedundant = rs.adjacentRings[i].isRedundant;
        pkt->setWidths(i, t);
    }

    // I'm a member of my other rings too: transition my own origin into them
    // now (using my adjacentRings for the width). Other members bridging the
    // same ring-pair also transition; the per-chain dup-guard absorbs overlap.
    forwardUpdateInterRing(pkt, ring);

    // Origin-UPDATE lives in the empty-chain inflight slot.
    std::vector<uint32_t> emptyChain;
    RingInflight& fl = rs.updateInflights[emptyChain];
    fl.seq_no = rs.originSeq;
    sendRingReliable(fl, primary, fallback, pkt, rxmtInterval, COLLECT_RETRY_LIMIT,
                     [this, ring, emptyChain]() { onUpdateBudgetExpired(ring, emptyChain); });

    MLOG_N_INFO(nbrs[0]) << "originate UPDATE ring='" << (char)ring
        << "' seq=" << rs.originSeq << " rr=" << rs.rr.size() << "\n";
}

void MyriadGP::insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t R, nodeid_t sender) {
    // 1. Append my (subnet, myId) pairs.
    size_t n = pkt->getRrArraySize();
    pkt->setRrArraySize(n + mySubnets.size());
    for (size_t i = 0; i < mySubnets.size(); ++i) {
        MyriadGpRr rr_entry;
        rr_entry.subnet = mySubnets[i];
        rr_entry.owner = myId;
        pkt->setRr(n + i, rr_entry);
    }

    // 2. Update transitions for each of my OTHER rings.
    for (uint32_t otherRing : selfRings) {
        if (otherRing == R) continue;
        bool found = false;
        for (size_t i = 0; i < pkt->getTransitionsArraySize(); ++i) {
            auto& t = pkt->getTransitionsForUpdate(i);
            if (t.ring != otherRing) continue;
            found = true;
            if (!t.isRedundant) t.isRedundant = true; // second contributor confirms double
            break;
        }
        if (!found) {
            size_t k = pkt->getTransitionsArraySize();
            pkt->setTransitionsArraySize(k + 1);
            MyriadGpRingTransition t;
            t.ring = otherRing;
            t.isRedundant = false;
            pkt->setTransitions(k, t);
        }
    }

    // 3. Forward to the OTHER ring-neighbor for R. A genuine 2-node ring has
    // no other neighbor: echo back to the sender so the king still gets its
    // circular ACK (with 2 spofs).
    nodeid_t to = otherRingNeighbor(R, sender);
    if (to == undefined_node) {
        if (rings[R].maxRingNeighbors <= 1) {
            to = sender;
            MLOG_N_DEBUG(sender) << "echo COLLECT ring='" << (char)R
                                 << "' (2-node ring)\n";
        } else {
            // using high-water ring neighbor count
            MLOG_N_WARN(sender) << "no other ring-neighbor for degraded ring='"
                                << (char)R << "'; dropping COLLECT\n";
            return;
        }
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Collect_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->encapsulate(pkt->dup());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward COLLECT ring='" << (char)R
                     << "' seq=" << pkt->getSeq_no()
                     << " king=" << (int)pkt->getRouter_id_src() << "\n";
}

void MyriadGP::handleUpdate(MyriadGpUpdate* update, nodeid_t sender) {
    uint32_t curRing = update->getCurrent_ring();
    nodeid_t king = update->getRouter_id_src();
    uint32_t seq = update->getSeq_no();

    // Chain is prepend-order; origin = chain.back(), or curRing if chain empty.
    size_t chainLen = update->getRing_chainArraySize();
    uint32_t origin = chainLen ? update->getRing_chain(chainLen - 1) : curRing;

    // Materialise the chain (the identity of this ring traversal).
    std::vector<uint32_t> chain;
    chain.reserve(chainLen);
    for (size_t i = 0; i < chainLen; ++i) chain.push_back(update->getRing_chain(i));

    RingState& curRs = rings[curRing];

    // Own ACK: my own traversal (this chain, in this ring) returned on the other
    // interface. Cancel that inflight, do NOT re-forward.
    if (king == myId) {
        auto it = curRs.updateInflights.find(chain);
        if (it != curRs.updateInflights.end() && it->second.tx != nullptr
                && seq == it->second.seq_no) {
            cancelRingReliable(it->second);
            curRs.updateInflights.erase(it);
            std::vector<uint32_t> fullChain;
            fullChain.push_back(curRing);
            for (uint32_t c : chain) fullChain.push_back(c);
            MLOG_N_DEBUG(sender) << "UPDATE circular-ACK ring='" << (char)curRing
                                 << "' seq=" << seq << " chain=" << formatChain(fullChain) << "\n";
        }
        return;
    }

    // Width trailer: only origin UPDATEs carry it, and its current_ring is the
    // ring the widths describe. Overwrite my stored widths for this ring (the
    // king's list is authoritative and complete).
    size_t widthLen = update->getWidthsArraySize();
    if (widthLen > 0) {
        curRs.adjacentRings.clear();
        curRs.adjacentRings.reserve(widthLen);
        for (size_t i = 0; i < widthLen; ++i) {
            auto& w = update->getWidths(i);
            curRs.adjacentRings.push_back({ w.ring, w.isRedundant });
        }
        MLOG_N_DEBUG(sender) << "stored widths for ring='" << (char)curRing
                             << "' (" << widthLen << " transitions)\n";
    }

    RingState& originRs = rings[origin];

    // Full path identity = current_ring :: packet chain. The packet chain omits
    // current_ring (a router never appends its own ring within the ring); we
    // prepend it so RIB entries are globally unique paths, not ring-relative.
    std::vector<uint32_t> fullChain;
    fullChain.reserve(chainLen + 1);
    fullChain.push_back(curRing);
    for (uint32_t c : chain) fullChain.push_back(c);

    // Decide whether to transition inter-ring. Regardless of the outcome we
    // still forward intra-ring at the end (needed for the circular ACK, and to
    // carry any handled-rings marks around the lap).
    bool doInterRing = true;

    // Dedup on (origin, fullChain): already forwarded at >= seq → intra-ring
    // forward is all that's needed.
    RibEntry* entry = nullptr;
    for (auto& e : originRs.ribIn) if (e.chain == fullChain) { entry = &e; break; }
    if (entry && entry->seq_no >= seq) {
        MLOG_N_DEBUG(sender) << "UPDATE dup origin='" << (char)origin
                             << "' seq=" << seq << " chain=" << formatChain(fullChain)
                             << " (stored " << entry->seq_no << ")\n";
        doInterRing = false;
    }

    if (doInterRing) {
        // Seq advance: newer generation for this origin → purge older chains and
        // refresh the authoritative rr from this packet.
        if (seq > originRs.curSeq) {
            originRs.curSeq = seq;
            originRs.ribIn.erase(
                std::remove_if(originRs.ribIn.begin(), originRs.ribIn.end(),
                    [seq](const RibEntry& e) { return e.seq_no < seq; }),
                originRs.ribIn.end());
            entry = nullptr;
            for (auto& e : originRs.ribIn) if (e.chain == fullChain) { entry = &e; break; }

            originRs.rr.clear();
            originRs.rr.reserve(update->getRrArraySize());
            for (size_t i = 0; i < update->getRrArraySize(); ++i)
                originRs.rr.push_back(update->getRr(i));
        }

        // Insert or replace the RIB entry for this chain.
        if (entry) {
            entry->seq_no = seq;
            entry->spof_count = update->getSpof_count();
        } else {
            originRs.ribIn.push_back({ fullChain, seq, update->getSpof_count() });
        }

        MLOG_N_INFO(sender) << "UPDATE store origin='" << (char)origin
                            << "' seq=" << seq << " chain=" << formatChain(fullChain)
                            << " spof=" << update->getSpof_count()
                            << " ribIn=" << originRs.ribIn.size() << "\n";

        // TODO: policy → preferred + materialized routes.

        // Loop prevention: if any of my rings is already in the packet chain,
        // I've been here — don't transition again (intra-ring forward still
        // happens below). Check the packet chain, NOT fullChain: current_ring
        // is always my ring.
        for (uint32_t myRing : selfRings)
            for (uint32_t c : chain) if (c == myRing) { doInterRing = false; break; }
    }

    // Transition into my other rings first — this marks handled_rings on the
    // packet so the intra-ring copy tells the other bridge not to duplicate.
    if (doInterRing) forwardUpdateInterRing(update, curRing);

    // Always forward within current_ring so the originator's lap can close,
    // carrying any handled-rings marks to downstream bridges.
    forwardUpdateInRing(update, curRing, sender);
}

void MyriadGP::forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender) {
    nodeid_t to = otherRingNeighbor(ring, sender);
    if (to == undefined_node) {
        // Genuine 2-node ring: echo back so the king's lap closes. Degraded
        // larger ring: drop, let the budget expire → DEGRADE.
        if (rings[ring].maxRingNeighbors > 1) return;
        to = sender;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Update_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->encapsulate(pkt->dup());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward UPDATE in-ring='" << (char)ring
                     << "' seq=" << pkt->getSeq_no() << "\n";
}

void MyriadGP::forwardUpdateInterRing(MyriadGpUpdate* pkt, uint32_t curRing) {
    size_t chainLen = pkt->getRing_chainArraySize();

    for (uint32_t R2 : selfRings) {
        if (R2 == curRing) continue;
        bool inChain = false;
        for (size_t i = 0; i < chainLen; ++i)
            if (pkt->getRing_chain(i) == R2) { inChain = true; break; }
        if (inChain) continue;

        // Another bridge of this transition already flooded R2 on this lap
        // (its mark rode the intra-ring copy to me) — don't duplicate.
        bool alreadyHandled = false;
        for (size_t i = 0; i < pkt->getHandled_ringsArraySize(); ++i)
            if (pkt->getHandled_rings(i) == R2) { alreadyHandled = true; break; }
        if (alreadyHandled) {
            MLOG_DEBUG() << "UPDATE transition into ring='" << (char)R2
                         << "' skipped: already flooded by another bridge this lap\n";
            continue;
        }

        RingState& rs2 = rings[R2];

        std::vector<nodeid_t> nbrs = ringNeighbors(R2);
        if (nbrs.empty()) {
            // No neighbor in R2 yet: transient at boot; a later round covers it.
            MLOG_DEBUG() << "UPDATE transition into ring='" << (char)R2
                         << "' skipped: no ring-neighbor\n";
            continue;
        }
        // 2-node target ring: single neighbor, probe mode (primary == fallback).
        nodeid_t primary = nbrs[0];
        nodeid_t fallback = nbrs.size() > 1 ? nbrs[1] : nbrs[0];

        // Prepend curRing to build the forwarded chain (also this traversal's key).
        std::vector<uint32_t> fwdChain;
        fwdChain.reserve(chainLen + 1);
        fwdChain.push_back(curRing);
        for (size_t i = 0; i < chainLen; ++i) fwdChain.push_back(pkt->getRing_chain(i));

        // A traversal for this exact chain may already be inflight in R2 (e.g. a
        // double transition where both bridges relay the same chain). Skip the dup.
        if (rs2.updateInflights.count(fwdChain)) {
            MLOG_N_DEBUG(nbrs[0]) << "UPDATE transition dup into ring='" << (char)R2
                                  << "' chain-len=" << fwdChain.size() << "; skipping\n";
            continue;
        }

        // Single-node transition into R2 is an unprotected point. Widths come
        // from this ring's origin UPDATE trailer (stored in adjacentRings);
        // unknown transitions default to single until that trailer arrives.
        bool doubleTransition = false;
        for (auto& a : rings[curRing].adjacentRings)
            if (a.ring == R2) { doubleTransition = a.isRedundant; break; }

        auto* fwd = new MyriadGpUpdate("Myriad_GP_Update");
        fwd->setType(MYRIADGP_UPDATE);
        fwd->setRouter_id_src(myId);          // I own this new ring traversal
        fwd->setSeq_no(pkt->getSeq_no());
        fwd->setCurrent_ring(R2);
        fwd->setSpof_count(pkt->getSpof_count() + (doubleTransition ? 0 : 1));

        fwd->setRing_chainArraySize(fwdChain.size());
        for (size_t i = 0; i < fwdChain.size(); ++i) fwd->setRing_chain(i, fwdChain[i]);

        // Carry the same rr.
        size_t rrLen = pkt->getRrArraySize();
        fwd->setRrArraySize(rrLen);
        for (size_t i = 0; i < rrLen; ++i) fwd->setRr(i, pkt->getRr(i));

        RingInflight& fl = rs2.updateInflights[fwdChain];
        fl.seq_no = pkt->getSeq_no();
        sendRingReliable(fl, primary, fallback, fwd, rxmtInterval, COLLECT_RETRY_LIMIT,
                         [this, R2, fwdChain]() { onUpdateBudgetExpired(R2, fwdChain); });

        MLOG_N_INFO(primary) << "UPDATE transition into ring='" << (char)R2
            << "' seq=" << pkt->getSeq_no() << " chain=" << formatChain(fwdChain)
            << " spof=" << fwd->getSpof_count() << "\n";

        // Mark R2 handled on the incoming packet so the intra-ring copy tells
        // the other bridge of this transition not to duplicate the flood.
        size_t h = pkt->getHandled_ringsArraySize();
        pkt->setHandled_ringsArraySize(h + 1);
        pkt->setHandled_rings(h, R2);
    }
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
    for (auto& [rid, currentRingState] : rings) {
        cancelRingReliable(currentRingState.collectInflight);
        for (auto& [chain, fl] : currentRingState.updateInflights)
            cancelRingReliable(fl);
        currentRingState.updateInflights.clear();
        if (currentRingState.refreshTimer) {
            cancelAndDelete(currentRingState.refreshTimer);
            currentRingState.refreshTimer = nullptr;
        }
        if (currentRingState.watchdogTimer) {
            cancelAndDelete(currentRingState.watchdogTimer);
            currentRingState.watchdogTimer = nullptr;
        }
    }
    rings.clear();
    degrades.clear();
    routes.clear();
    cancelHelloTimer();
}

void MyriadGP::finish() {
    // TODO: dump path-vector table.
}

} // namespace routingsim
