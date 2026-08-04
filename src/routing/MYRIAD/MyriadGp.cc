#include "routing/MYRIAD/MyriadGp.h"
#include <algorithm>
#include <cmath>
#include <limits>

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
    ringCostExponent = par("ringCostExponent");
    holdInterval = par("holdInterval");

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
        // 2-node-ring support DISABLED for now (assume every ring has >=2
        // members). Originate only once BOTH ring-neighbors are two-way, so a
        // boot-transient single-neighbor view can't crown a premature king.
        // To re-enable degenerate 2-node rings, restore the `count >= 1` bound
        // (see [[0014-two-node-ring-echo]]) and the echo sites below.
        // if (count >= 1 && count <= 2 && rings[R].collectInflight.tx == nullptr) {
        if (count == 2 && rings[R].collectInflight.tx == nullptr) {
            originateCollect(R, nbr.id);
        } else if (count > 2) {
            MLOG_N_WARN(nbr.id) << "ring '" << R
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
    pkt->setMember_count(1); // forwarders increment

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

    MLOG_N_INFO(to) << "originate COLLECT ring='" << ring
        << "' seq=" << currentRingState.originSeq << "\n";
}

void MyriadGP::reoriginateCollect(uint32_t ring) {
    // Pick either ring-neighbor as the send target (alternation covers the
    // other direction on retransmit); skip if the ring isn't fully seen.
    std::vector<nodeid_t> nbrs = ringNeighbors(ring);
    if (nbrs.empty()) {
        MLOG_DEBUG() << "re-collect ring='" << ring
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
    rs.refreshTimer = addOneShotTimer(refreshInterval + uniform(0, refreshJitter), [this, ring](short) {
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
        MLOG_INFO() << "watchdog fired for ring='" << ring
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
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(pkt->dup());
    countSend(sendTypeLabel(pkt->getName()));
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
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(h->pkt->dup());
    countSend(sendTypeLabel(h->pkt->getName()));
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
    MLOG_WARN() << "ring send budget expired ring='" << ring
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
    MLOG_WARN() << "UPDATE traversal budget expired ring='" << ring
                << "' chain-len=" << chain.size() << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onBatchBudgetExpired(uint32_t ring, uint32_t batchLapSeq) {
    auto rit = rings.find(ring);
    if (rit == rings.end()) return;
    auto it = rit->second.batchInflights.find(batchLapSeq);
    if (it == rit->second.batchInflights.end()) return;
    it->second.budgetTimer = nullptr;   // see onCollectBudgetExpired
    cancelRingReliable(it->second);
    rit->second.batchInflights.erase(it);
    MLOG_WARN() << "batch-lap budget expired ring='" << ring
                << "' seq=" << batchLapSeq << " — ring broken\n";
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
        MLOG_N_INFO(nbr.id) << "grace COLLECT ring='" << R
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
        case MYRIADGP_UPDATE_BATCH:
            handleUpdateBatch(check_and_cast<MyriadGpUpdateBatch*>(header), sender);
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

            // Lap closed: member_count now equals the ring's router count.
            currentRingState.memberCount = collect->getMember_count();

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

            MLOG_INFO() << "COLLECT complete ring='" << R
                        << "' seq=" << seq
                        << " rr=" << currentRingState.rr.size()
                        << " transitions=[";
            for (size_t i = 0; i < currentRingState.adjacentRings.size(); ++i) {
                auto& a = currentRingState.adjacentRings[i];
                EV << (i ? "," : "") << a.ring << (a.isRedundant ? "=D" : "=S");
            }
            EV << "]\n";

            originateUpdate(R);

            // Winning the lap makes me (or keeps me) king: run the refresh
            // timer, stop watching for someone else.
            if (!currentRingState.isKing) {
                currentRingState.isKing = true;
                MLOG_INFO() << "became king of ring='" << R << "'\n";
            }
            startRefresh(R);
        } else {
            MLOG_N_WARN(sender) << "stale own COLLECT return ring='" << R
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
            MLOG_N_INFO(sender) << "yield COLLECT ring='" << R
                                << "' to king=" << (int)king << "\n";
            cancelCollect(R);                     // no-op if no inflight
            if (currentRingState.isKing) cancelKingship(R);
            insertAndForwardCollect(collect, R, sender);
        } else {
            // Ignore lower-id king; my collection/kingship stands.
            MLOG_N_DEBUG(sender) << "ignore COLLECT ring='" << R
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
        MLOG_WARN() << "originate UPDATE ring='" << ring
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
    pkt->setChain_sizesArraySize(0);      // parallel to ring_chain
    pkt->setMember_count(rs.memberCount); // origin ring size (trailer)
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
    // (hold=0 sends now; hold>0 queues for the next batch). handled accrues the
    // rings I fan into so sibling bridges on the origin lap don't duplicate.
    std::vector<uint32_t> handled;
    std::vector<uint32_t> emptyChain, emptySizes;
    forwardChainInterRing(ring, rs.originSeq, 0, emptyChain, emptySizes, rs.rr, handled);
    pkt->setHandled_ringsArraySize(handled.size());
    for (size_t i = 0; i < handled.size(); ++i) pkt->setHandled_rings(i, handled[i]);

    // Origin-UPDATE lives in the empty-chain inflight slot.
    RingInflight& fl = rs.updateInflights[emptyChain];
    fl.seq_no = rs.originSeq;
    sendRingReliable(fl, primary, fallback, pkt, rxmtInterval, COLLECT_RETRY_LIMIT,
                     [this, ring, emptyChain]() { onUpdateBudgetExpired(ring, emptyChain); });

    MLOG_N_INFO(nbrs[0]) << "originate UPDATE ring='" << ring
        << "' seq=" << rs.originSeq << " rr=" << rs.rr.size() << "\n";
}

void MyriadGP::insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t R, nodeid_t sender) {
    // Count myself as a ring member
    pkt->setMember_count(pkt->getMember_count() + 1);

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
        // 2-node-ring echo DISABLED for now (see onTwoWayReached). No other
        // ring-neighbor → drop; boot retransmits recover once the ring is up.
        // To re-enable, restore the maxRingNeighbors<=1 echo branch:
        // if (rings[R].maxRingNeighbors <= 1) {
        //     to = sender;
        //     MLOG_N_DEBUG(sender) << "echo COLLECT ring='" << R << "' (2-node ring)\n";
        // } else {
            MLOG_N_WARN(sender) << "no other ring-neighbor for ring='"
                                << R << "'; dropping COLLECT\n";
            return;
        // }
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Collect_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(pkt->dup());
    countSend("collect");
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward COLLECT ring='" << R
                     << "' seq=" << pkt->getSeq_no()
                     << " king=" << (int)pkt->getRouter_id_src() << "\n";
}

double MyriadGP::pathCost(const RibEntry& e) const {
    // sum up member_count^a over the chain
    double cost = 0.0;
    for (uint32_t s : e.chain_sizes) cost += std::pow((double)s, ringCostExponent);
    return cost;
}

void MyriadGP::selectPreferred(RingState& rs) {
    // Strict lexicographic best: min spof, min pathCost, min chain length, then chain
    // lexicographic if draw.
    // TODO check: Linear scan over ribIn but ribIn is small per origin. If a ring
    // ever accumulates many chains, index by a sorted key instead.
    rs.preferred = SIZE_MAX;
    double bestCost = 0.0;
    for (size_t i = 0; i < rs.ribIn.size(); ++i) {
        const RibEntry& e = rs.ribIn[i];
        double cost = pathCost(e);
        if (rs.preferred == SIZE_MAX) { rs.preferred = i; bestCost = cost; continue; }
        const RibEntry& b = rs.ribIn[rs.preferred];
        bool better =
            e.spof_count != b.spof_count ? e.spof_count < b.spof_count :
            cost != bestCost             ? cost < bestCost :
            e.chain.size() != b.chain.size() ? e.chain.size() < b.chain.size() :
            e.chain < b.chain;
        if (better) { rs.preferred = i; bestCost = cost; }
    }
}

void MyriadGP::handleUpdate(MyriadGpUpdate* update, nodeid_t sender) {
    uint32_t curRing = update->getCurrent_ring();
    nodeid_t king = update->getRouter_id_src();
    uint32_t seq = update->getSeq_no();

    // Chain is prepend-order; origin = chain.back(), or curRing if chain empty.
    size_t chainLen = update->getRing_chainArraySize();

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
            MLOG_N_DEBUG(sender) << "UPDATE circular-ACK ring='" << curRing
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
        MLOG_N_DEBUG(sender) << "stored widths for ring='" << curRing
                             << "' (" << widthLen << " transitions)\n";
    }
    // Origin UPDATE also carries this rings member count (its size)
    if (chainLen == 0 && update->getMember_count() > 0)
        curRs.memberCount = update->getMember_count();

    // rr snapshot carried by this packet (used on seq-advance inside processChain).
    std::vector<MyriadGpRr> rr;
    rr.reserve(update->getRrArraySize());
    for (size_t i = 0; i < update->getRrArraySize(); ++i) rr.push_back(update->getRr(i));

    // chain_sizes carried (parallel to packet chain; processChain prepends curRing's).
    std::vector<uint32_t> sizes;
    sizes.reserve(update->getChain_sizesArraySize());
    for (size_t i = 0; i < update->getChain_sizesArraySize(); ++i)
        sizes.push_back(update->getChain_sizes(i));

    // Process this chain: RIB/select, and (if winner + loop-free) transition it
    // inter-ring. handled accrues the rings it fans into.
    std::vector<uint32_t> handled;
    for (size_t i = 0; i < update->getHandled_ringsArraySize(); ++i)
        handled.push_back(update->getHandled_rings(i));
    processChain(curRing, seq, update->getSpof_count(), chain, sizes, rr, &rr, handled);

    // Carry accrued handled_rings back onto the packet for the intra-ring lap.
    update->setHandled_ringsArraySize(handled.size());
    for (size_t i = 0; i < handled.size(); ++i) update->setHandled_rings(i, handled[i]);

    // Always forward within current_ring so the originator's lap can close,
    // carrying any handled-rings marks to downstream bridges.
    forwardUpdateInRing(update, curRing, sender);
}

// Process ONE learned chain at this node: RIB insert/select + inter-ring
// transition of the winner. Extracted from handleUpdate so handleUpdateBatch can
// call it per-entry. `handled` is seeded with the lapping packet's marks and
// gains any rings this chain fans into. originRrTrailer is the rr to adopt on a
// seq-advance (the packet's own rr).
void MyriadGP::processChain(uint32_t curRing, uint32_t seq, uint32_t spof,
                            const std::vector<uint32_t>& chain,
                            const std::vector<uint32_t>& chainSizes,
                            const std::vector<MyriadGpRr>& rr,
                            const std::vector<MyriadGpRr>* originRrTrailer,
                            std::vector<uint32_t>& handled) {
    RingState& curRs = rings[curRing];
    uint32_t origin = chain.empty() ? curRing : chain.back();
    RingState& originRs = rings[origin];

    // Full path identity = current_ring :: packet chain.
    std::vector<uint32_t> fullChain;
    fullChain.reserve(chain.size() + 1);
    fullChain.push_back(curRing);
    for (uint32_t c : chain) fullChain.push_back(c);

    // Sizes parallel to fullChain.
    std::vector<uint32_t> fullSizes;
    fullSizes.reserve(chain.size() + 1);
    fullSizes.push_back(curRs.memberCount ? curRs.memberCount : UNKNOWN_RING_SIZE);
    for (uint32_t s : chainSizes) fullSizes.push_back(s);

    bool doInterRing = true;

    // Dedup on (origin, fullChain): already forwarded at >= seq → nothing to do.
    RibEntry* entry = nullptr;
    for (auto& e : originRs.ribIn) if (e.chain == fullChain) { entry = &e; break; }
    if (entry && entry->seq_no >= seq) {
        MLOG_DEBUG() << "UPDATE dup origin='" << origin
                     << "' seq=" << seq << " chain=" << formatChain(fullChain)
                     << " (stored " << entry->seq_no << ")\n";
        doInterRing = false;
    }

    if (doInterRing) {
        // Seq advance: newer generation → purge older chains, refresh rr.
        if (seq > originRs.curSeq) {
            originRs.curSeq = seq;
            originRs.ribIn.erase(
                std::remove_if(originRs.ribIn.begin(), originRs.ribIn.end(),
                    [seq](const RibEntry& e) { return e.seq_no < seq; }),
                originRs.ribIn.end());
            entry = nullptr;
            for (auto& e : originRs.ribIn) if (e.chain == fullChain) { entry = &e; break; }
            if (originRrTrailer) originRs.rr = *originRrTrailer;
        }

        // Insert or replace the RIB entry for this chain.
        if (entry) {
            entry->seq_no = seq;
            entry->spof_count = spof;
            entry->chain_sizes = fullSizes;
        } else {
            originRs.ribIn.push_back({ fullChain, fullSizes, seq, spof });
        }

        MLOG_INFO() << "UPDATE store origin='" << origin
                    << "' seq=" << seq << " chain=" << formatChain(fullChain)
                    << " spof=" << spof << " ribIn=" << originRs.ribIn.size() << "\n";

        // Best-path: only the winner transitions inter-ring.
        selectPreferred(originRs);
        bool isWinner = originRs.preferred != SIZE_MAX
                        && originRs.ribIn[originRs.preferred].chain == fullChain;
        if (!isWinner) {
            MLOG_DEBUG() << "UPDATE not preferred origin='" << origin
                         << "' chain=" << formatChain(fullChain) << " — stored, not propagated\n";
            doInterRing = false;
        }

        // Loop prevention: any of my rings already in the packet chain → been here.
        for (uint32_t myRing : selfRings)
            for (uint32_t c : chain) if (c == myRing) { doInterRing = false; break; }
    }

    if (doInterRing)
        forwardChainInterRing(curRing, seq, spof, chain, chainSizes, rr, handled);
}

void MyriadGP::forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender) {
    nodeid_t to = otherRingNeighbor(ring, sender);
    if (to == undefined_node) {
        // 2-node-ring echo DISABLED for now (see onTwoWayReached). No other
        // ring-neighbor → drop (a degraded larger ring drops here too, letting
        // the budget expire → DEGRADE). To re-enable degenerate 2-node rings,
        // restore the echo-to-sender branch:
        // if (rings[ring].maxRingNeighbors > 1) return;
        // to = sender;
        return;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Update_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(pkt->dup());
    countSend("update");
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward UPDATE in-ring='" << ring
                     << "' seq=" << pkt->getSeq_no() << "\n";
}

MyriadGpUpdate* MyriadGP::buildTransitionUpdate(uint32_t R2, nodeid_t src, uint32_t seq,
                              uint32_t spof, const std::vector<uint32_t>& fwdChain,
                              const std::vector<uint32_t>& fwdSizes,
                              const std::vector<MyriadGpRr>& rr) {
    auto* fwd = new MyriadGpUpdate("Myriad_GP_Update");
    fwd->setType(MYRIADGP_UPDATE);
    fwd->setRouter_id_src(src);            // owner of this new ring traversal
    fwd->setSeq_no(seq);
    fwd->setCurrent_ring(R2);
    fwd->setSpof_count(spof);
    fwd->setRing_chainArraySize(fwdChain.size());
    for (size_t i = 0; i < fwdChain.size(); ++i) fwd->setRing_chain(i, fwdChain[i]);
    fwd->setChain_sizesArraySize(fwdSizes.size());
    for (size_t i = 0; i < fwdSizes.size(); ++i) fwd->setChain_sizes(i, fwdSizes[i]);
    fwd->setRrArraySize(rr.size());
    for (size_t i = 0; i < rr.size(); ++i) fwd->setRr(i, rr[i]);
    return fwd;
}

void MyriadGP::forwardChainInterRing(uint32_t curRing, uint32_t seq, uint32_t spof,
                          const std::vector<uint32_t>& chain,
                          const std::vector<uint32_t>& chainSizes,
                          const std::vector<MyriadGpRr>& rr,
                          std::vector<uint32_t>& handled) {
    for (uint32_t R2 : selfRings) {
        if (R2 == curRing) continue;
        // R2 already in the chain → transitioning there would loop; skip.
        bool inChain = false;
        for (uint32_t c : chain) if (c == R2) { inChain = true; break; }
        if (inChain) continue;
        // Already flooded into R2 on this lap (a sibling bridge's mark rode the
        // lapping packet/entry to me) — don't duplicate.
        bool already = false;
        for (uint32_t h : handled) if (h == R2) { already = true; break; }
        if (already) continue;

        std::vector<nodeid_t> nbrs = ringNeighbors(R2);
        if (nbrs.empty()) {
            MLOG_DEBUG() << "transition into ring='" << R2
                         << "' skipped: no ring-neighbor\n";
            continue;
        }
        nodeid_t primary = nbrs[0];
        nodeid_t fallback = nbrs.size() > 1 ? nbrs[1] : nbrs[0];

        // fwdChain = curRing :: chain (the new traversal's identity in R2).
        std::vector<uint32_t> fwdChain;
        fwdChain.reserve(chain.size() + 1);
        fwdChain.push_back(curRing);
        for (uint32_t c : chain) fwdChain.push_back(c);

        // Single-node transition is an unprotected point (+1 spof) unless the
        // width trailer marked this transition redundant.
        bool doubleTransition = false;
        for (auto& a : rings[curRing].adjacentRings)
            if (a.ring == R2) { doubleTransition = a.isRedundant; break; }
        uint32_t fwdSpof = spof + (doubleTransition ? 0 : 1);

        // chain_sizes parallel to fwdChain: prepend curRing's size.
        std::vector<uint32_t> fwdSizes;
        fwdSizes.reserve(fwdChain.size());
        uint32_t curSize = rings[curRing].memberCount;
        fwdSizes.push_back(curSize ? curSize : UNKNOWN_RING_SIZE);
        for (uint32_t s : chainSizes) fwdSizes.push_back(s);

        if (holdInterval > SIMTIME_ZERO) {
            // hold>0: queue for the next batch flush. RIB dedup upstream + the
            // per-entry handled marks prevent double-queue, so no reserved slot.
            pendingTransitions[{R2, primary}].push_back(
                PendingTransition{R2, primary, fallback, seq, fwdSpof, fwdChain, fwdSizes, rr});
            if (!holdTimer)
                holdTimer = addOneShotTimer(holdInterval, [this](short) {
                    holdTimer = nullptr;
                    flushPendingTransitions();
                    return false;
                });
            MLOG_N_DEBUG(primary) << "transition HELD into ring='" << R2
                << "' chain=" << formatChain(fwdChain) << " spof=" << fwdSpof << "\n";
        } else {
            // hold=0: single-chain transition sent NOW (exact pre-batch path,
            // including the same-chain dup-guard on updateInflights).
            if (rings[R2].updateInflights.count(fwdChain)) {
                MLOG_N_DEBUG(primary) << "transition dup into ring='" << R2
                                      << "' chain-len=" << fwdChain.size() << "; skipping\n";
                continue;
            }
            auto* fwd = buildTransitionUpdate(R2, myId, seq, fwdSpof, fwdChain, fwdSizes, rr);
            RingInflight& fl = rings[R2].updateInflights[fwdChain];
            fl.seq_no = seq;
            sendRingReliable(fl, primary, fallback, fwd, rxmtInterval, COLLECT_RETRY_LIMIT,
                             [this, R2, fwdChain]() { onUpdateBudgetExpired(R2, fwdChain); });
            MLOG_N_INFO(primary) << "transition into ring='" << R2
                << "' seq=" << seq << " chain=" << formatChain(fwdChain)
                << " spof=" << fwdSpof << "\n";
        }
        handled.push_back(R2);
    }
}

void MyriadGP::flushPendingTransitions() {
    auto buckets = std::move(pendingTransitions);
    pendingTransitions.clear();

    for (auto& [key, chains] : buckets) {
        uint32_t R2 = key.first;
        nodeid_t primary = key.second;
        RingState& rs2 = rings[R2];

        // Re-validate each held chain against current state; collect survivors.
        std::vector<PendingTransition*> live;
        for (auto& pt : chains) {
            uint32_t origin = pt.fwdChain.back();
            RingState& originRs = rings[origin];
            bool valid = originRs.preferred != SIZE_MAX
                && originRs.ribIn[originRs.preferred].chain == pt.fwdChain
                && originRs.ribIn[originRs.preferred].seq_no == pt.seq;
            // Loop check: none of my rings may already sit in the packet chain
            // (fwdChain minus its head, which is curRing = one of my rings).
            for (size_t i = 1; valid && i < pt.fwdChain.size(); ++i)
                for (uint32_t myRing : selfRings)
                    if (pt.fwdChain[i] == myRing) { valid = false; break; }
            if (!valid) {
                MLOG_DEBUG() << "held transition dropped (superseded/loop) ring='"
                             << R2 << "' chain=" << formatChain(pt.fwdChain) << "\n";
                continue;
            }
            live.push_back(&pt);
        }
        if (live.empty()) continue;

        // One batch, owned by me, that laps R2 WHOLE under a single circular-ACK.
        nodeid_t fallback = live.front()->fallback;
        uint32_t lapSeq = ++rs2.batchLapSeq;

        auto* batch = new MyriadGpUpdateBatch("Myriad_GP_Update_Batch");
        batch->setType(MYRIADGP_UPDATE_BATCH);
        batch->setRouter_id_src(myId);        // owner: recognised on own-return
        batch->setCurrent_ring(R2);
        batch->setSeq_no(lapSeq);             // my per-ring batch-lap counter
        batch->setEntriesArraySize(live.size());
        for (size_t i = 0; i < live.size(); ++i) {
            PendingTransition* pt = live[i];
            MyriadGpUpdateEntry e;
            e.setSeq_no(pt->seq);
            e.setSpof_count(pt->spof);
            e.setRing_chainArraySize(pt->fwdChain.size());
            for (size_t j = 0; j < pt->fwdChain.size(); ++j) e.setRing_chain(j, pt->fwdChain[j]);
            e.setChain_sizesArraySize(pt->fwdSizes.size());
            for (size_t j = 0; j < pt->fwdSizes.size(); ++j) e.setChain_sizes(j, pt->fwdSizes[j]);
            e.setRrArraySize(pt->rr.size());
            for (size_t j = 0; j < pt->rr.size(); ++j) e.setRr(j, pt->rr[j]);
            batch->setEntries(i, e);          // handled_rings empty at birth
        }
        batchEntriesTotal += live.size();
        batchesCreated += 1;
        if (live.size() > batchMaxEntries) batchMaxEntries = live.size();

        RingInflight& fl = rs2.batchInflights[lapSeq];
        fl.seq_no = lapSeq;
        sendRingReliable(fl, primary, fallback, batch, rxmtInterval, COLLECT_RETRY_LIMIT,
                         [this, R2, lapSeq]() { onBatchBudgetExpired(R2, lapSeq); });
        MLOG_N_INFO(primary) << "flush UPDATE batch into ring='" << R2
            << "' seq=" << lapSeq << " entries=" << live.size() << "\n";
    }
}

void MyriadGP::forwardBatchInRing(MyriadGpUpdateBatch* pkt, uint32_t ring, nodeid_t sender) {
    nodeid_t to = otherRingNeighbor(ring, sender);
    if (to == undefined_node) {
        // 2-node-ring echo DISABLED for now (see forwardUpdateInRing). No other
        // ring-neighbor → drop; the batch-lap budget expires → ring broken.
        return;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Update_Batch_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    np->encapsulate(pkt->dup());
    countSend("update_batch");
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward batch in-ring='" << ring
                     << "' seq=" << pkt->getSeq_no() << "\n";
}

void MyriadGP::handleUpdateBatch(MyriadGpUpdateBatch* batch, nodeid_t sender) {
    uint32_t R2 = batch->getCurrent_ring();
    nodeid_t owner = batch->getRouter_id_src();
    uint32_t lapSeq = batch->getSeq_no();
    RingState& rs2 = rings[R2];

    // Own ACK: my batch lapped R2 and returned. Cancel the inflight; the batch
    // is NOT unpacked or re-forwarded (its entries were processed on the way out).
    if (owner == myId) {
        auto it = rs2.batchInflights.find(lapSeq);
        if (it != rs2.batchInflights.end() && it->second.tx != nullptr
                && lapSeq == it->second.seq_no) {
            cancelRingReliable(it->second);
            rs2.batchInflights.erase(it);
            MLOG_N_DEBUG(sender) << "batch circular-ACK ring='" << R2
                                 << "' seq=" << lapSeq
                                 << " entries=" << batch->getEntriesArraySize() << "\n";
        }
        return;
    }

    // Per-entry: RIB/select + this entry's own inter-ring transitions (which
    // become NEW held batches into MY other rings). Update the entry's
    // handled_rings in place so the lapping batch tells sibling bridges what I
    // already flooded — but the batch itself stays WHOLE for the lap.
    for (size_t i = 0; i < batch->getEntriesArraySize(); ++i) {
        MyriadGpUpdateEntry& e = batch->getEntriesForUpdate(i);
        std::vector<uint32_t> chain;
        for (size_t j = 0; j < e.getRing_chainArraySize(); ++j) chain.push_back(e.getRing_chain(j));
        std::vector<uint32_t> sizes;
        for (size_t j = 0; j < e.getChain_sizesArraySize(); ++j) sizes.push_back(e.getChain_sizes(j));
        std::vector<MyriadGpRr> rr;
        for (size_t j = 0; j < e.getRrArraySize(); ++j) rr.push_back(e.getRr(j));
        std::vector<uint32_t> handled;
        for (size_t j = 0; j < e.getHandled_ringsArraySize(); ++j)
            handled.push_back(e.getHandled_rings(j));

        processChain(R2, e.getSeq_no(), e.getSpof_count(), chain, sizes, rr, &rr, handled);

        e.setHandled_ringsArraySize(handled.size());
        for (size_t j = 0; j < handled.size(); ++j) e.setHandled_rings(j, handled[j]);
    }

    // Forward the WHOLE batch within R2 so the owner's lap closes.
    forwardBatchInRing(batch, R2, sender);
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
        for (auto& [seq, fl] : currentRingState.batchInflights)
            cancelRingReliable(fl);
        currentRingState.batchInflights.clear();
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
    pendingTransitions.clear();
    if (holdTimer) { cancelAndDelete(holdTimer); holdTimer = nullptr; }
    cancelHelloTimer();
}

void MyriadGP::finish() {
    // TODO: dump path-vector table.
    recordSendScalars();
    recordScalar("batch_entries_total", batchEntriesTotal);
    recordScalar("batch_max_entries", batchMaxEntries);
    recordScalar("batches_created", batchesCreated);
}

} // namespace routingsim
