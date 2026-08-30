#include "routing/MYRIAD/MyriadGp.h"
#include "routing/MYRIAD/MyriadPduSize.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>

namespace routingsim {

Define_Module(MyriadGP);

namespace {
using namespace msgsize;

int64_t gpCollectBytes(const MyriadGpCollect* p) {
    return MYRIADGP_HDR + U32 + U32 + 2 * BOOL             // member_count + lap max + crashed bit + kind
         + (int64_t) p->getRrArraySize() * RR
         + (int64_t) p->getTransitionsArraySize() * TRANSITION;
}
int64_t gpUpdateBytes(const MyriadGpUpdate* p) {
    return MYRIADGP_HDR + U32 + U32 + U64                  // member_count + spof_count + chain_cost
         + (int64_t) p->getRrArraySize() * RR
         + (int64_t) p->getRing_chainArraySize() * U32
         + (int64_t) p->getHandled_ringsArraySize() * U32
         + (int64_t) p->getWidthsArraySize() * TRANSITION;
}
int64_t gpBatchBytes(const MyriadGpUpdateBatch* p) {
    int64_t b = MYRIADGP_HDR;
    for (size_t i = 0; i < p->getEntriesArraySize(); ++i) {
        const MyriadGpUpdateEntry& e = p->getEntries(i);
        b += U32 + U32 + U64                               // seq_no + spof_count + chain_cost
           + (int64_t) e.getRing_chainArraySize() * U32
           + (int64_t) e.getHandled_ringsArraySize() * U32
           + (int64_t) e.getRrArraySize() * RR;
    }
    return b;
}

static uint64_t satAdd(uint64_t a, uint64_t b) {
    return (b > std::numeric_limits<uint64_t>::max() - a) ? std::numeric_limits<uint64_t>::max() : a + b;
}
} // namespace

uint32_t MyriadGP::memberCountOf(uint32_t ring) const {
    auto it = memberships.find(ring);
    return it == memberships.end() ? 0 : it->second.memberCount;
}

void MyriadGP::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADGP_PACKET_TYPE;

    node().bind(MYRIADGP_PACKET_TYPE, this);

    mySubnets = { (static_cast<uint32_t>(myId) << 8) | 0x01,
                  (static_cast<uint32_t>(myId) << 8) | 0x02 };

    refreshInterval = par("refreshInterval");
    refreshJitter = par("refreshJitter");
    watchdogMargin = par("watchdogMargin");
    updateJitter = par("updateJitter");
    rib.configure(par("ringCostExponent"), par("rankUseSpof"), par("maxEcmpPaths"));
    degradeEnabled = par("degradeEnabled");
    recoveryReplayEnabled = par("recoveryReplayEnabled");
    refreshFloodsUpdate = par("refreshFloodsUpdate");
    carryRr = par("carryRr");
    maxBatchBytes = par("maxBatchBytes");
    holdInterval = par("holdInterval");
    snapshotInterval = par("snapshotInterval");
    snapshotStats = par("snapshotFormat").stdstringValue() == "stats";
    collectWarmupDuration = par("collectWarmupDuration");
    collectWarmupInterval = par("collectWarmupInterval");
    // 0s (default) means derive: ~3 COLLECTs per ring over the warmup window.
    // Tying the cadence to the warmup length keeps ring-dense topologies from
    // flooding COLLECTs faster than their laps can close (see the .ned).
    if (collectWarmupInterval <= SIMTIME_ZERO)
        collectWarmupInterval = 0.3 * collectWarmupDuration;

    MLOG_INFO() << "MyriadGP up\n";

    // Periodically send hellos to all neighbors. Cold start staggers the first
    // hello over joinJitter so formation isn't a synchronized herd, mirroring the
    // reboot spread; a reboot's stagger already comes from the failure scenario.
    scheduleHello(helloInterval + uniform(0, joinJitter));

    if (snapshotInterval > 0 && !par("ribDumpFile").stdstringValue().empty())
        startSnapshot();

    if (collectWarmupDuration > 0) {
        startWarmupCollects();
    }
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
        RingMembership& m = memberships[R];
        // The high-water mark separates formation from repair: it only grows, so
        // a count we have never reached means the ring is still coming up, while
        // a mate merely returning cannot exceed it. A survivor stays silent on a
        // repair — the standing king's refresh re-announces the ring, and if the
        // king itself died a watchdog coup elects a new one.
        bool grew = count > m.maxRingNeighbors;
        if (grew) m.maxRingNeighbors = count;
        // Originate once >=1 ring-neighbor is two-way (a genuine 2-node ring has
        // exactly one and closes its lap via echo; see the echo sites below).
        if (grew && count <= 2 && m.collectInflight.tx == nullptr) {
            // reset() wipes memberships, so a rebooted node always "grows". It
            // probes first; only a closed lap is worth a replay (handleCollect).
            if (rebooted) {
                originateCollect(R, nbr.id, COLLECT_PROBE);  // reboot: immediate, recovery latency matters
            } else if (collectWarmupInterval > 0) {
                // with helloJitter/joinJitter at 0 every node's first hello fires
                // together, so every ring would originate its election COLLECT at
                // the same instant. Spread the first election over uniform(0, collectWarmupInterval)
                uint32_t r = R;
                addOneShotTimer(uniform(0, collectWarmupInterval),
                                [this, r](short) { reoriginateCollect(r); return false; });
            } else {
                originateCollect(R, nbr.id, COLLECT_ELECTION);
            }
        } else if (count > 2) {
            MLOG_N_WARN(nbr.id) << "ring '" << R
                << "' has >2 ring-neighbors (misconfig); ignoring\n";
        }
    }
}

void MyriadGP::originateCollect(uint32_t ring, nodeid_t to, MyriadGpCollectKind kind) {
    RingMembership& m = memberships[ring];
    m.originSeq += 1;
    m.collectInflight.seq_no = m.originSeq;

    auto* pkt = new MyriadGpCollect("Myriad_GP_Collect");
    pkt->setType(MYRIADGP_COLLECT);
    pkt->setRouter_id_src(myId);     // king_id
    pkt->setSeq_no(m.originSeq);
    pkt->setCurrent_ring(ring);
    pkt->setMember_count(1); // forwarders increment

    // Stateless (fresh) originator loses the king election to any live member.
    pkt->setOriginator_crashed(m.memberCount == 0);
    pkt->setLap_max_routes(rib.knownOrigins());
    pkt->setKind(kind);

    // Insert my (subnet, myId) pairs.
    if (carryRr) {
        pkt->setRrArraySize(mySubnets.size());
        for (size_t i = 0; i < mySubnets.size(); ++i) {
            MyriadGpRr rr_entry; rr_entry.subnet = mySubnets[i];
            rr_entry.owner = myId;
            pkt->setRr(i, rr_entry);
        }
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
    pkt->setByteLength(gpCollectBytes(pkt));
    sendRingReliable(m.collectInflight, to, to, pkt, rxmtInterval,
                     COLLECT_RETRY_LIMIT, [this, ring]() { onCollectBudgetExpired(ring); });

    MLOG_N_INFO(to) << "originate COLLECT ring='" << ring
        << "' seq=" << m.originSeq << "\n";
}

void MyriadGP::reoriginateCollect(uint32_t ring, bool routine) {
    // Pick either ring-neighbor as the send target (alternation covers the
    // other direction on retransmit); skip if the ring isn't fully seen.
    std::vector<nodeid_t> nbrs = ringNeighbors(ring);
    if (nbrs.empty()) {
        MLOG_DEBUG() << "re-collect ring='" << ring
                     << "' skipped: no ring-neighbors\n";
        return;
    }
    if (memberships[ring].collectInflight.tx != nullptr) return; // one at a time
    memberships[ring].routineRefreshPending = routine;
    originateCollect(ring, nbrs[0]);
}

void MyriadGP::startRefresh(uint32_t ring) {
    RingMembership& rs = memberships[ring];
    if (rs.watchdogTimer) { cancelAndDelete(rs.watchdogTimer); rs.watchdogTimer = nullptr; }
    if (rs.refreshTimer)  { cancelAndDelete(rs.refreshTimer);  rs.refreshTimer = nullptr; }
    rs.refreshTimer = addOneShotTimer(refreshInterval + uniform(0, refreshJitter), [this, ring](short) {
        memberships[ring].refreshTimer = nullptr;
        reoriginateCollect(ring, /*routine=*/true);
        if (memberships[ring].isKing) startRefresh(ring); // reschedule
        return false;
    });
}

void MyriadGP::startSnapshot() {
    // Re-arms itself like startRefresh; purely observational (see header).
    snapshotTimer = addOneShotTimer(snapshotInterval, [this](short) {
        snapshotTimer = nullptr;
        std::ofstream f(par("ribDumpFile").stdstringValue(), std::ios::app);
        dumpRibTo(f, "t=" + simTime().str() + " ", snapshotStats);
        startSnapshot();
        return false;
    });
}

void MyriadGP::startWarmupCollects() {
    addOneShotTimer(collectWarmupInterval, [this](short) -> bool {
        // Spread each king's re-COLLECT uniformly across the interval instead of
        // firing them all at once: a synchronized herd every tick is itself a
        // flood spike on ring-dense topologies. uniform(0, interval) = 0.3*warmup.
        for (auto& [ring, m] : memberships)
            if (m.isKing) {
                uint32_t r = ring;
                addOneShotTimer(uniform(0, collectWarmupInterval),
                                [this, r](short) { reoriginateCollect(r); return false; });
            }
        if (simTime() < collectWarmupDuration)
            startWarmupCollects();
        return false;
    });
}

void MyriadGP::startWatchdog(uint32_t ring) {
    RingMembership& rs = memberships[ring];
    if (rs.watchdogTimer) { cancelAndDelete(rs.watchdogTimer); rs.watchdogTimer = nullptr; }
    // Fire just after a healthy king's refresh SHOULD have re-armed us.
    simtime_t delay = refreshInterval + refreshJitter + watchdogMargin;
    rs.watchdogTimer = addOneShotTimer(delay, [this, ring](short) {
        memberships[ring].watchdogTimer = nullptr;
        MLOG_INFO() << "watchdog fired for ring='" << ring
                    << "' — starting coup\n";
        reoriginateCollect(ring); // become king only on own-return
        return false;
    });
}

void MyriadGP::cancelKingship(uint32_t ring) {
    RingMembership& rs = memberships[ring];
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
    countSend(sendTypeLabel(pkt->getName()), np->getByteLength());
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
    countSend(sendTypeLabel(h->pkt->getName()), np->getByteLength());
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
    auto it = memberships.find(ring);
    if (it == memberships.end()) return;
    cancelRingReliable(it->second.collectInflight);
}

void MyriadGP::replayIfBest(MyriadGpCollect* lap, uint32_t ring) {
    if (rib.knownOrigins() <= lap->getLap_max_routes()) return;
    if (lap->getKind() != COLLECT_REPLAY || !recoveryReplayEnabled
        || simTime() < collectWarmupDuration || memberCountOf(ring) == 0) {
        lap->setLap_max_routes(rib.knownOrigins());
        return;
    }
    MLOG_INFO() << "replaying into ring='" << ring << "' (lap carries "
                << lap->getLap_max_routes() << " < " << rib.knownOrigins() << ")\n";
    replayInto(ring);
    lap->setLap_max_routes(rib.knownOrigins());
}

void MyriadGP::onCollectBudgetExpired(uint32_t ring) {
    RingMembership& m = memberships[ring];
    // This runs from the budgetTimer's own callback; the harness frees that
    // message after we return, so drop our handle before cancelRingReliable
    // (which would otherwise cancelAndDelete the message we're executing).
    m.collectInflight.budgetTimer = nullptr;
    if (m.collectInflight.tx == nullptr) return; // already resolved
    cancelRingReliable(m.collectInflight);
    MLOG_WARN() << "ring send budget expired ring='" << ring
                << "' seq=" << m.collectInflight.seq_no << " — ring broken\n";
    // Degrade only on actual degradation
    if (degradeEnabled && ringNeighbors(ring).size() < m.maxRingNeighbors)
        originateDegrade(ring);
}

void MyriadGP::onUpdateBudgetExpired(uint32_t ring, std::vector<uint32_t> chain) {
    auto rit = memberships.find(ring);
    if (rit == memberships.end()) return;
    auto& infl = rit->second.updateInflights;
    auto it = infl.find(chain);
    if (it == infl.end()) return;
    // Drop the budget-timer handle first (see onCollectBudgetExpired).
    it->second.budgetTimer = nullptr;
    cancelRingReliable(it->second);
    infl.erase(it);
    MLOG_WARN() << "UPDATE traversal budget expired ring='" << ring
                << "' chain-len=" << chain.size() << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onBatchBudgetExpired(uint32_t ring, uint32_t batchLapSeq) {
    auto rit = memberships.find(ring);
    if (rit == memberships.end()) return;
    auto& infl = rit->second.batchInflights;
    auto it = infl.find(batchLapSeq);
    if (it == infl.end()) return;
    it->second.budgetTimer = nullptr;   // see onCollectBudgetExpired
    cancelRingReliable(it->second);
    infl.erase(it);
    MLOG_WARN() << "batch-lap budget expired ring='" << ring
                << "' seq=" << batchLapSeq << " — ring broken\n";
    // TODO: originate DEGRADE for this ring (beacon mode).
}

void MyriadGP::onNeighborDead(GpNeighbor& nbr, bool /*wasFull*/) {
    // For each ring shared with the dead neighbor: if no inflight for that
    // ring, probe the dead neighbor. Its retransmits act as the liveness grace
    // period — the probe closing means the ring is whole, the budget expiring
    // fires DEGRADE. A probe never crowns a king or re-announces the ring, so a
    // failure costs no UPDATE; that is the refresh's job.
    // If an inflight already exists, do nothing and let its budget run.
    for (uint32_t R : nbr.sharedRings) {
        if (memberships[R].collectInflight.tx != nullptr) continue;
        MLOG_N_INFO(nbr.id) << "probe COLLECT ring='" << R
                            << "' to dead neighbor\n";
        originateCollect(R, nbr.id, COLLECT_PROBE);
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

bool MyriadGP::collectWins(bool incomingCrashed, nodeid_t incomingId, uint32_t ring) const {
    bool myCrashed = memberCountOf(ring) == 0;
    if (incomingCrashed != myCrashed) return myCrashed;   // live beats crashed
    return incomingId > myId;
}

void MyriadGP::handleCollect(MyriadGpCollect* collect, nodeid_t sender) {
    uint32_t R = collect->getCurrent_ring();
    nodeid_t king = collect->getRouter_id_src();
    uint32_t seq = collect->getSeq_no();
    RingMembership& m = memberships[R];

    // Own return — circular ACK completes the collection.
    if (king == myId) {
        if (m.collectInflight.tx != nullptr && seq == m.collectInflight.seq_no) {
            cancelCollect(R);

            // Lap closed: member_count now equals the ring's router count.
            m.memberCount = collect->getMember_count();

            // Snapshot the collected data onto ring state.
            std::vector<MyriadGpRr> collected;
            collected.reserve(collect->getRrArraySize());
            for (size_t i = 0; i < collect->getRrArraySize(); ++i)
                collected.push_back(collect->getRr(i));
            rib.setRr(R, std::move(collected));

            m.adjacentRings.clear();
            m.adjacentRings.reserve(collect->getTransitionsArraySize());
            for (size_t i = 0; i < collect->getTransitionsArraySize(); ++i) {
                auto& t = collect->getTransitions(i);
                m.adjacentRings.push_back({ t.ring, t.isRedundant });
            }

            MLOG_INFO() << "COLLECT complete ring='" << R
                        << "' seq=" << seq
                        << " rr=" << rib.rr(R).size()
                        << " transitions=[";
            for (size_t i = 0; i < m.adjacentRings.size(); ++i) {
                auto& a = m.adjacentRings[i];
                EV << (i ? "," : "") << a.ring << (a.isRedundant ? "=D" : "=S");
            }
            EV << "]\n";

            // Lap closed, so the ring is whole and a replay reaches every
            // member. Terminates: a lap reporting nothing new asks for nothing.
            if (recoveryReplayEnabled && simTime() >= collectWarmupDuration
                && collect->getLap_max_routes() > rib.knownOrigins())
                originateCollect(R, sender, COLLECT_REPLAY);

            // Only an election lap crowns a king and re-announces the ring. A
            // probe or replay lap has already done its job above: it proved the
            // ring whole and restored my member size / rr / transitions.
            if (collect->getKind() != COLLECT_ELECTION) {
                MLOG_INFO() << (collect->getKind() == COLLECT_PROBE ? "probe" : "replay")
                            << " COLLECT complete ring='" << R << "' seq=" << seq << "\n";
                // The lap doubles as a king probe: any king it passed cleared the
                // flag. Still set means the king co-failed with me, so nothing
                // re-announces this ring until a watchdog fires a whole refresh
                // period from now. Coup now instead.
                if (collect->getOriginator_crashed()) {
                    MLOG_INFO() << "replay lap met no king on ring='" << R << "' — coup\n";
                    reoriginateCollect(R);
                } else if (!m.isKing && !m.watchdogTimer) {
                    startWatchdog(R);  // rebooted node returns a plain member
                }
                return;
            }

            // Winning the lap makes me (or keeps me) king. The very first
            // origination per ring is jittered: all rings' COLLECTs settle at
            // roughly the same time, so firing UPDATEs immediately storms the
            // network. Refresh re-originations are already desynced by
            // refreshJitter, so only the first (isKing still false) is delayed.
            if (!m.isKing) {
                m.isKing = true;
                MLOG_INFO() << "became king of ring='" << R << "'\n";
                if (updateJitter > 0)
                    addOneShotTimer(uniform(0, updateJitter),
                                    [this, R](short) { originateUpdate(R); return false; });
                else
                    originateUpdate(R);
            } else {
                bool routine = m.routineRefreshPending;
                m.routineRefreshPending = false;
                if (refreshFloodsUpdate || !routine)
                    originateUpdate(R);
            }

            startRefresh(R);
        } else {
            MLOG_N_WARN(sender) << "stale own COLLECT return ring='" << R
                                << "' seq=" << seq << " (cur=" << m.collectInflight.seq_no << ")\n";
        }
        return;
    }

    // Arm a watchdog the moment I'm a non-king member, however I learned
    // the ring. Only an ELECTION lap RESETS it.
    if (!m.isKing) {
        if (collect->getKind() == COLLECT_ELECTION || !m.watchdogTimer)
            startWatchdog(R);
    }

    // Non-election laps are forwarded UNCONDITIONALLY (never suppressed by
    // contention) so they always reach the originator.
    if (collect->getKind() != COLLECT_ELECTION) {
        if (m.isKing) collect->setOriginator_crashed(false);  // king alive: no coup
        insertAndForwardCollect(collect, R, sender);
        return;
    }

    bool crashed = collect->getOriginator_crashed();

    // Election (formation, refresh, or coup). Priority is (live, id): a live node
    // beats a crashed one, else higher id wins. I yield/demote to a winner, or hold
    // and ignore a loser (my next collect makes them yield).
    if (m.collectInflight.tx != nullptr || m.isKing) {
        if (collectWins(crashed, king, R)) {
            MLOG_N_INFO(sender) << "yield COLLECT ring='" << R
                                << "' to king=" << (int)king << "\n";
            cancelCollect(R);                     // no-op if no inflight
            if (m.isKing) cancelKingship(R);
            insertAndForwardCollect(collect, R, sender);
        } else {
            MLOG_N_DEBUG(sender) << "ignore COLLECT ring='" << R
                                 << "' from king=" << (int)king << "\n";
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

bool MyriadGP::isTwoNodeRing(uint32_t ring) {
    // Echo Path check
    size_t claimants = 0;
    for (auto& [nid, n] : neighbors)
        for (uint32_t r : n.sharedRings) if (r == ring) { ++claimants; break; }
    return claimants <= 1;
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
    RingMembership& m = memberships[ring];
    rib.installSelfChain(ring, m.originSeq,
        rib.sizeToCost(m.memberCount ? m.memberCount : UNKNOWN_RING_SIZE));
    const std::vector<MyriadGpRr>& rr = rib.rr(ring);

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
    pkt->setSeq_no(m.originSeq);          // same seq as the collect round
    pkt->setCurrent_ring(ring);
    pkt->setRing_chainArraySize(0);       // origin: empty chain
    pkt->setChain_cost(0);
    pkt->setMember_count(m.memberCount);  // origin ring size (trailer)
    pkt->setSpof_count(0);

    pkt->setRrArraySize(rr.size());
    for (size_t i = 0; i < rr.size(); ++i) pkt->setRr(i, rr[i]);

    // Trailer: my (the king's) transition widths for this ring, so members can
    // set spof_count correctly when they transition this update onward.
    pkt->setWidthsArraySize(m.adjacentRings.size());
    for (size_t i = 0; i < m.adjacentRings.size(); ++i) {
        MyriadGpRingTransition t;
        t.ring = m.adjacentRings[i].ring;
        t.isRedundant = m.adjacentRings[i].isRedundant;
        pkt->setWidths(i, t);
    }

    // I'm a member of my other rings too: transition my own origin into them
    // (hold=0 sends now; hold>0 queues for the next batch). handled accrues the
    // rings I fan into so sibling bridges on the origin lap don't duplicate.
    std::vector<uint32_t> handled;
    std::vector<uint32_t> emptyChain;
    forwardChainInterRing(ring, m.originSeq, 0, emptyChain, 0, rr, handled);
    pkt->setHandled_ringsArraySize(handled.size());
    for (size_t i = 0; i < handled.size(); ++i) pkt->setHandled_rings(i, handled[i]);

    // Origin-UPDATE lives in the empty-chain inflight slot.
    pkt->setByteLength(gpUpdateBytes(pkt));
    RingInflight& fl = m.updateInflights[emptyChain];
    fl.seq_no = m.originSeq;
    sendRingReliable(fl, primary, fallback, pkt, rxmtInterval, COLLECT_RETRY_LIMIT,
                     [this, ring, emptyChain]() { onUpdateBudgetExpired(ring, emptyChain); });

    MLOG_N_INFO(nbrs[0]) << "originate UPDATE ring='" << ring
        << "' seq=" << m.originSeq << " rr=" << rr.size() << "\n";
}

void MyriadGP::insertAndForwardCollect(MyriadGpCollect* pkt, uint32_t R, nodeid_t sender) {
    RingMembership& m = memberships[R];
    replayIfBest(pkt, R);
    // Count myself as a ring member
    pkt->setMember_count(pkt->getMember_count() + 1);

    // 1. Append my (subnet, myId) pairs.
    if (carryRr) {
        size_t n = pkt->getRrArraySize();
        pkt->setRrArraySize(n + mySubnets.size());
        for (size_t i = 0; i < mySubnets.size(); ++i) {
            MyriadGpRr rr_entry;
            rr_entry.subnet = mySubnets[i];
            rr_entry.owner = myId;
            pkt->setRr(n + i, rr_entry);
        }
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
        // Genuine 2-node ring: echo back to the sender so the king still gets
        // its circular ACK. A degraded larger ring (had >1 neighbor) instead
        // drops, letting the budget expire → DEGRADE.
        if (isTwoNodeRing(R)) {
            to = sender;
            MLOG_N_DEBUG(sender) << "echo COLLECT ring='" << R << "' (2-node ring)\n";
        } else {
            MLOG_N_WARN(sender) << "no other ring-neighbor for ring='"
                                << R << "'; dropping COLLECT\n";
            return;
        }
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Collect_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    pkt->setByteLength(gpCollectBytes(pkt));
    np->encapsulate(pkt->dup());
    countSend("collect", np->getByteLength());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward COLLECT ring='" << R
                     << "' seq=" << pkt->getSeq_no()
                     << " king=" << (int)pkt->getRouter_id_src() << "\n";
}

void MyriadGP::originateDegrade(uint32_t ring) {
    auto [seq, beacon] = rib.markDegraded(ring);
    floodDegrade(ring, seq, beacon, undefined_node);
}

void MyriadGP::floodDegrade(uint32_t ring, uint32_t seq, uint32_t beacon, nodeid_t skip) {
    auto tmpl = std::make_unique<MyriadGpDegrade>("Myriad_GP_Degrade");
    tmpl->setType(MYRIADGP_DEGRADE);
    tmpl->setRouter_id_src(myId);
    tmpl->setRing_id(ring);
    tmpl->setSeq_no(seq);
    tmpl->setBeacon_no(beacon);
    tmpl->setByteLength(msgsize::TYPE_TAG + msgsize::NODEID + 3 * msgsize::U32);
    for (auto& [nid, nbr] : neighbors) {
        if (nid == skip) continue;
        auto np = std::make_unique<NodePkt>("Myriad_GP_Degrade_Pkt");
        np->setType(MYRIADGP_PACKET_TYPE);
        np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
        np->encapsulate(tmpl->dup());
        countSend("degrade", np->getByteLength());
        sendToNode(nid, std::move(np));
    }
}

void MyriadGP::handleUpdate(MyriadGpUpdate* update, nodeid_t sender) {
    uint32_t curRing = update->getCurrent_ring();
    nodeid_t king = update->getRouter_id_src();
    uint32_t seq = update->getSeq_no();

    size_t chainLen = update->getRing_chainArraySize();
    std::vector<uint32_t> chain;
    chain.reserve(chainLen);
    for (size_t i = 0; i < chainLen; ++i) chain.push_back(update->getRing_chain(i));

    RingMembership& m = memberships[curRing];

    // Own ACK: my traversal of this chain returned on the other interface.
    if (king == myId) {
        auto it = m.updateInflights.find(chain);
        if (it != m.updateInflights.end() && it->second.tx != nullptr
                && seq == it->second.seq_no) {
            cancelRingReliable(it->second);
            m.updateInflights.erase(it);
            std::vector<uint32_t> fullChain;
            fullChain.push_back(curRing);
            for (uint32_t c : chain) fullChain.push_back(c);
            MLOG_N_DEBUG(sender) << "UPDATE circular-ACK ring='" << curRing
                                 << "' seq=" << seq << " chain=" << formatChain(fullChain) << "\n";
        }
        return;
    }

    // Width trailer rides only origin UPDATEs; the king's list is authoritative,
    // so overwrite ours for this ring.
    size_t widthLen = update->getWidthsArraySize();
    if (widthLen > 0) {
        m.adjacentRings.clear();
        m.adjacentRings.reserve(widthLen);
        for (size_t i = 0; i < widthLen; ++i) {
            auto& w = update->getWidths(i);
            m.adjacentRings.push_back({ w.ring, w.isRedundant });
        }
        MLOG_N_DEBUG(sender) << "stored widths for ring='" << curRing
                             << "' (" << widthLen << " transitions)\n";
    }
    // Origin UPDATE also carries this ring's size.
    if (chainLen == 0 && update->getMember_count() > 0) {
        if (m.memberCount > 0 && m.memberCount != update->getMember_count())
            rib.invalidate(curRing);
        m.memberCount = update->getMember_count();
    }

    std::vector<MyriadGpRr> rr;
    rr.reserve(update->getRrArraySize());
    for (size_t i = 0; i < update->getRrArraySize(); ++i) rr.push_back(update->getRr(i));

    std::vector<uint32_t> handled;
    for (size_t i = 0; i < update->getHandled_ringsArraySize(); ++i)
        handled.push_back(update->getHandled_rings(i));
    processChain(curRing, seq, update->getSpof_count(), chain, update->getChain_cost(), rr, &rr, handled);

    // Carry accrued handled_rings back onto the packet for the intra-ring lap.
    update->setHandled_ringsArraySize(handled.size());
    for (size_t i = 0; i < handled.size(); ++i) update->setHandled_rings(i, handled[i]);

    // Always forward within current_ring so the originator's lap can close.
    forwardUpdateInRing(update, curRing, sender);
}

// originRrTrailer is the rr to adopt on a seq-advance (the packet's own rr).
// See the declaration in MyriadGp.h for the rest.
void MyriadGP::processChain(uint32_t curRing, uint32_t seq, uint32_t spof,
                            const std::vector<uint32_t>& chain,
                            uint64_t chainCost,
                            const std::vector<MyriadGpRr>& rr,
                            const std::vector<MyriadGpRr>* originRrTrailer,
                            std::vector<uint32_t>& handled) {
    uint32_t origin = chain.empty() ? curRing : chain.back();

    // The gates below and the loop guard stay here rather than in the RIB: they
    // read membership and selfRings, which are protocol state.

    // Don't store or bridge an inter-ring update arriving on a ring whose COLLECT
    // hasn't landed here yet (the dup guard blocks re-forwarding). Excludes seq==0
    // replay: it's adopted into the RIB and never re-forwarded, so the dup-guard
    // rationale doesn't apply
    if (!chain.empty() && seq != 0 && memberCountOf(curRing) == 0) {
        MLOG_DEBUG() << "inter-ring UPDATE on ring='" << curRing
                     << "' deferred: own COLLECT not yet complete\n";
        return;
    }

    // Full path identity = current_ring :: packet chain, with sizes parallel.
    std::vector<uint32_t> fullChain;
    fullChain.reserve(chain.size() + 1);
    fullChain.push_back(curRing);
    for (uint32_t c : chain) fullChain.push_back(c);

    uint32_t curSize = memberCountOf(curRing);
    uint64_t fullCost = satAdd(rib.sizeToCost(curSize ? curSize : UNKNOWN_RING_SIZE), chainCost);

    StoreResult r = rib.store(origin, fullChain, fullCost, seq, spof, originRrTrailer);
    if (r.dup) {
        MLOG_DEBUG() << "UPDATE co-best dup origin='" << origin
                     << "' seq=" << seq << " chain=" << formatChain(fullChain) << "\n";
        return;
    }
    if (!r.propagate) return;

    MLOG_INFO() << "UPDATE store origin='" << origin
                << "' seq=" << seq << " chain=" << formatChain(fullChain)
                << " spof=" << spof << " cobest=" << r.cobest << "\n";

    // Loop prevention: any of my rings already in the chain means I bridged
    // this origin's flood on an earlier lap, so don't re-bridge it now.
    for (uint32_t myRing : selfRings)
        for (uint32_t c : chain) if (c == myRing) return;

    // Only the current generation propagates
    if (seq >= rib.curSeq(origin))
        forwardChainInterRing(curRing, seq, spof, chain, chainCost, rr, handled);
}

void MyriadGP::forwardUpdateInRing(MyriadGpUpdate* pkt, uint32_t ring, nodeid_t sender) {
    nodeid_t to = otherRingNeighbor(ring, sender);
    if (to == undefined_node) {
        // Genuine 2-node ring: echo to the sender to close the lap. A degraded
        // larger ring drops, letting the budget expire → DEGRADE.
        if (!isTwoNodeRing(ring)) return;
        to = sender;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Update_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    pkt->setByteLength(gpUpdateBytes(pkt));
    np->encapsulate(pkt->dup());
    countSend("update", np->getByteLength());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward UPDATE in-ring='" << ring
                     << "' seq=" << pkt->getSeq_no() << "\n";
}

MyriadGpUpdate* MyriadGP::buildTransitionUpdate(uint32_t R2, nodeid_t src, uint32_t seq,
                              uint32_t spof, const std::vector<uint32_t>& fwdChain,
                              uint64_t fwdCost,
                              const std::vector<MyriadGpRr>& rr) {
    auto* fwd = new MyriadGpUpdate("Myriad_GP_Update");
    fwd->setType(MYRIADGP_UPDATE);
    fwd->setRouter_id_src(src);            // owner of this new ring traversal
    fwd->setSeq_no(seq);
    fwd->setCurrent_ring(R2);
    fwd->setSpof_count(spof);
    fwd->setRing_chainArraySize(fwdChain.size());
    for (size_t i = 0; i < fwdChain.size(); ++i) fwd->setRing_chain(i, fwdChain[i]);
    fwd->setChain_cost(fwdCost);
    fwd->setRrArraySize(rr.size());
    for (size_t i = 0; i < rr.size(); ++i) fwd->setRr(i, rr[i]);
    fwd->setByteLength(gpUpdateBytes(fwd));
    return fwd;
}

void MyriadGP::forwardChainInterRing(uint32_t curRing, uint32_t seq, uint32_t spof,
                          const std::vector<uint32_t>& chain,
                          uint64_t chainCost,
                          const std::vector<MyriadGpRr>& rr,
                          std::vector<uint32_t>& handled) {
    // During COLLECT warmup suppress inter-ring forwarding; memberCounts are
    // still being established
    if (collectWarmupDuration > 0 && simTime() < collectWarmupDuration)
        return;
    for (uint32_t R2 : selfRings) {
        if (R2 == curRing) continue;
        // R2 in the chain would loop; already in `handled` means a sibling bridge
        // flooded it on this lap. Skip both.
        bool inChain = false;
        for (uint32_t c : chain) if (c == R2) { inChain = true; break; }
        if (inChain) continue;
        bool already = false;
        for (uint32_t h : handled) if (h == R2) { already = true; break; }
        if (already) continue;

        // R2's COLLECT hasn't landed here yet: its members can't bridge out of it
        // (same gate in processChain) target ring has to prove redundancy first.
        if (memberCountOf(R2) == 0) {
            MLOG_DEBUG() << "transition into ring='" << R2
                         << "' skipped: its COLLECT not yet complete\n";
            continue;
        }

        std::vector<nodeid_t> nbrs = ringNeighbors(R2);
        if (nbrs.empty()) {
            MLOG_DEBUG() << "transition into ring='" << R2
                         << "' skipped: no ring-neighbor\n";
            continue;
        }
        nodeid_t primary = nbrs[0];
        nodeid_t fallback = nbrs.size() > 1 ? nbrs[1] : nbrs[0];

        std::vector<uint32_t> fwdChain;   // curRing :: chain: identity in R2
        fwdChain.reserve(chain.size() + 1);
        fwdChain.push_back(curRing);
        for (uint32_t c : chain) fwdChain.push_back(c);

        // +1 spof unless the width trailer marked this transition redundant.
        bool doubleTransition = false;
        for (auto& a : memberships[curRing].adjacentRings)
            if (a.ring == R2) { doubleTransition = a.isRedundant; break; }
        uint32_t fwdSpof = spof + (doubleTransition ? 0 : 1);

        uint32_t curSize = memberCountOf(curRing);
        uint64_t fwdCost = satAdd(rib.sizeToCost(curSize ? curSize : UNKNOWN_RING_SIZE), chainCost);

        if (holdInterval > SIMTIME_ZERO) {
            // Queue for the next flush; RIB dedup + handled marks prevent double-queue.
            pendingTransitions[{R2, primary}].push_back(
                PendingTransition{R2, primary, fallback, seq, fwdSpof, fwdChain, fwdCost, rr});
            if (!holdTimer)
                holdTimer = addOneShotTimer(holdInterval, [this](short) {
                    holdTimer = nullptr;
                    flushPendingTransitions();
                    return false;
                });
            MLOG_N_DEBUG(primary) << "transition HELD into ring='" << R2
                << "' chain=" << formatChain(fwdChain) << " spof=" << fwdSpof << "\n";
        } else {
            // hold=0: send now, with the same-chain dup-guard on updateInflights.
            RingMembership& m2 = memberships[R2];
            if (m2.updateInflights.count(fwdChain)) {
                MLOG_N_DEBUG(primary) << "transition dup into ring='" << R2
                                      << "' chain-len=" << fwdChain.size() << "; skipping\n";
                continue;
            }
            auto* fwd = buildTransitionUpdate(R2, myId, seq, fwdSpof, fwdChain, fwdCost, rr);
            RingInflight& fl = m2.updateInflights[fwdChain];
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
        RingMembership& rs2 = memberships[R2];

        // Send ALL queued, no re-validation: a chain that was winner-on-arrival
        // already propagated downstream, and via loop prevention may be the only
        // path to some nodes. Re-dropping non-final winners here loses those routes.
        std::vector<PendingTransition*> live;
        live.reserve(chains.size());
        for (auto& pt : chains) live.push_back(&pt);
        if (live.empty()) continue;

        nodeid_t fallback = live.front()->fallback;

        // Pack into batches up to maxBatchBytes; overflow spills into more that
        // lap R2 independently. A lone over-cap entry still ships (can't split a
        // chain). Modelled per-entry size: chain*8 + rr*6 + 16, ~16B header.
        auto entryBytes = [](const PendingTransition* pt) {
            return (long)(pt->fwdChain.size() * 8 + pt->rr.size() * 6 + 16);
        };
        size_t i = 0;
        while (i < live.size()) {
            std::vector<PendingTransition*> group;
            long bytes = 16;   // batch header
            while (i < live.size()
                   && (group.empty() || bytes + entryBytes(live[i]) <= maxBatchBytes)) {
                group.push_back(live[i]);
                bytes += entryBytes(live[i]);
                ++i;
            }
            uint32_t lapSeq = ++rs2.batchLapSeq;
            auto* batch = new MyriadGpUpdateBatch("Myriad_GP_Update_Batch");
            batch->setType(MYRIADGP_UPDATE_BATCH);
            batch->setRouter_id_src(myId);        // owner: recognised on own-return
            batch->setCurrent_ring(R2);
            batch->setSeq_no(lapSeq);             // per-ring batch-lap counter
            batch->setEntriesArraySize(group.size());
            for (size_t g = 0; g < group.size(); ++g) {
                PendingTransition* pt = group[g];
                MyriadGpUpdateEntry e;
                e.setSeq_no(pt->seq);
                e.setSpof_count(pt->spof);
                e.setRing_chainArraySize(pt->fwdChain.size());
                for (size_t j = 0; j < pt->fwdChain.size(); ++j) e.setRing_chain(j, pt->fwdChain[j]);
                e.setChain_cost(pt->fwdCost);
                e.setRrArraySize(pt->rr.size());
                for (size_t j = 0; j < pt->rr.size(); ++j) e.setRr(j, pt->rr[j]);
                batch->setEntries(g, e);          // handled_rings empty at birth
            }
            batchEntriesTotal += group.size();
            batchesCreated += 1;
            if (group.size() > batchMaxEntries) batchMaxEntries = group.size();

            RingInflight& fl = rs2.batchInflights[lapSeq];
            fl.seq_no = lapSeq;
            batch->setByteLength(gpBatchBytes(batch));
            sendRingReliable(fl, primary, fallback, batch, rxmtInterval, COLLECT_RETRY_LIMIT,
                             [this, R2, lapSeq]() { onBatchBudgetExpired(R2, lapSeq); });
            MLOG_N_INFO(primary) << "flush UPDATE batch into ring='" << R2
                << "' seq=" << lapSeq << " entries=" << group.size()
                << " bytes=" << bytes << "\n";
        }
    }
}

void MyriadGP::replayInto(uint32_t ring) {
    std::vector<nodeid_t> nbrs = ringNeighbors(ring);
    if (nbrs.empty()) return;
    nodeid_t primary = nbrs[0];
    nodeid_t fallback = nbrs.size() > 1 ? nbrs[1] : nbrs[0];
    RingMembership& rs = memberships[ring];

    // A replay is a plain seq==0 UPDATE, so the receiver prepends `ring` and adds
    // its cost like any inter-ring transition (see processChain). We therefore
    // send what lies BEYOND `ring`: as king I am a converged member of it, so
    // rebasing my whole RIB onto it hands the recovered node a route to every
    // origin I know, whichever of my rings I actually reach it through.
    // ponytail: a chain transiting a ring the recovered node is itself in gives it
    // a via-`ring` detour where a direct entry exists. Not special-cased — that
    // ring's own replay lap delivers the direct chain, which is strictly cheaper
    // and wins selectPreferred.
    struct RebasedChain { std::vector<uint32_t> chain; uint64_t cost; uint32_t spof;
                          const std::vector<MyriadGpRr>* rr; };
    std::vector<RebasedChain> replay;
    uint64_t ringCost = rib.sizeToCost(memberCountOf(ring));

    for (auto& e : rib.preferredChains()) {
        auto rit = std::find(e.chain->rbegin(), e.chain->rend(), ring);
        if (rit != e.chain->rend()) {
            // Send what lies beyond the LAST occurrence of `ring`; anything
            // before it is a detour out of the ring and back. [ring] itself
            // reduces to an empty chain = origin==current_ring at the receiver.
            size_t cut = e.chain->size() - (size_t)(rit - e.chain->rbegin());
            // Exact only if the ring was the front; a mid-cut keeps the full
            // cost/spof, over-stating both, which is the safe direction.
            bool exact = (cut == 1);
            replay.push_back({{e.chain->begin() + cut, e.chain->end()},
                              exact && e.cost > ringCost ? e.cost - ringCost : e.cost,
                              e.spof, e.rr});
        } else {
            // I bridge this one in myself: +1 spof for the ring->front hop unless
            // the transition is redundant, mirroring forwardChainInterRing.
            bool redundant = false;
            for (auto& a : rs.adjacentRings)
                if (a.ring == e.chain->front()) { redundant = a.isRedundant; break; }
            replay.push_back({*e.chain, e.cost, e.spof + (redundant ? 0 : 1), e.rr});
        }
    }
    if (replay.empty()) return;

    auto entryBytes = [](const RebasedChain& e) {
        return (long)(e.chain.size() * 8 + e.rr->size() * 6 + 16);
    };

    // Pack into batches up to maxBatchBytes (see flushPendingTransitions); each
    // laps `ring` under its own circular-ACK.
    size_t i = 0;
    while (i < replay.size()) {
        std::vector<const RebasedChain*> group;
        long bytes = 16;
        while (i < replay.size()
               && (group.empty() || bytes + entryBytes(replay[i]) <= maxBatchBytes)) {
            group.push_back(&replay[i]);
            bytes += entryBytes(replay[i]);
            ++i;
        }
        uint32_t lapSeq = ++rs.batchLapSeq;
        auto* batch = new MyriadGpUpdateBatch("Myriad_GP_Update_Batch");
        batch->setType(MYRIADGP_UPDATE_BATCH);
        batch->setRouter_id_src(myId);
        batch->setCurrent_ring(ring);
        batch->setSeq_no(lapSeq);
        batch->setEntriesArraySize(group.size());
        for (size_t g = 0; g < group.size(); ++g) {
            const RebasedChain& e = *group[g];
            const std::vector<MyriadGpRr>& rr = *e.rr;
            MyriadGpUpdateEntry en;
            en.setSeq_no(0);                      // sentinel: replay, adopt-if-empty
            en.setSpof_count(e.spof);
            en.setRing_chainArraySize(e.chain.size());
            for (size_t j = 0; j < e.chain.size(); ++j) en.setRing_chain(j, e.chain[j]);
            en.setChain_cost(e.cost);
            en.setRrArraySize(rr.size());
            for (size_t j = 0; j < rr.size(); ++j) en.setRr(j, rr[j]);
            batch->setEntries(g, en);
        }
        batchEntriesTotal += group.size();
        batchesCreated += 1;
        if (group.size() > batchMaxEntries) batchMaxEntries = group.size();

        RingInflight& fl = rs.batchInflights[lapSeq];
        fl.seq_no = lapSeq;
        batch->setByteLength(gpBatchBytes(batch));
        sendRingReliable(fl, primary, fallback, batch, rxmtInterval, COLLECT_RETRY_LIMIT,
                         [this, ring, lapSeq]() { onBatchBudgetExpired(ring, lapSeq); });
        MLOG_N_INFO(primary) << "replay batch into ring='" << ring
            << "' seq=" << lapSeq << " entries=" << group.size() << "\n";
    }
}

void MyriadGP::forwardBatchInRing(MyriadGpUpdateBatch* pkt, uint32_t ring, nodeid_t sender) {
    nodeid_t to = otherRingNeighbor(ring, sender);
    if (to == undefined_node) {
        // Genuine 2-node ring: echo to the sender to close the batch lap (see
        // forwardUpdateInRing). A degraded larger ring drops → budget expiry.
        if (!isTwoNodeRing(ring)) return;
        to = sender;
    }
    auto np = std::make_unique<NodePkt>("Myriad_GP_Update_Batch_Pkt");
    np->setType(MYRIADGP_PACKET_TYPE);
    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
    pkt->setByteLength(gpBatchBytes(pkt));
    np->encapsulate(pkt->dup());
    countSend("update_batch", np->getByteLength());
    sendToNode(to, std::move(np));
    MLOG_N_DEBUG(to) << "forward batch in-ring='" << ring
                     << "' seq=" << pkt->getSeq_no() << "\n";
}

void MyriadGP::handleUpdateBatch(MyriadGpUpdateBatch* batch, nodeid_t sender) {
    uint32_t R2 = batch->getCurrent_ring();
    nodeid_t owner = batch->getRouter_id_src();
    uint32_t lapSeq = batch->getSeq_no();
    RingMembership& rs2 = memberships[R2];

    // Own ACK: my batch lapped R2 and returned. Its entries were processed on
    // the way out, so don't unpack or re-forward.
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

    // Process each entry (its transitions become new held batches into my other
    // rings), updating its handled_rings in place so sibling bridges downstream
    // see what I flooded. The batch itself stays WHOLE for the lap.
    for (size_t i = 0; i < batch->getEntriesArraySize(); ++i) {
        MyriadGpUpdateEntry& e = batch->getEntriesForUpdate(i);
        std::vector<uint32_t> chain;
        for (size_t j = 0; j < e.getRing_chainArraySize(); ++j) chain.push_back(e.getRing_chain(j));
        std::vector<MyriadGpRr> rr;
        for (size_t j = 0; j < e.getRrArraySize(); ++j) rr.push_back(e.getRr(j));
        std::vector<uint32_t> handled;
        for (size_t j = 0; j < e.getHandled_ringsArraySize(); ++j)
            handled.push_back(e.getHandled_rings(j));

        processChain(R2, e.getSeq_no(), e.getSpof_count(), chain, e.getChain_cost(), rr, &rr, handled);

        e.setHandled_ringsArraySize(handled.size());
        for (size_t j = 0; j < handled.size(); ++j) e.setHandled_rings(j, handled[j]);
    }

    // Forward the WHOLE batch within R2 so the owner's lap closes.
    forwardBatchInRing(batch, R2, sender);
}

void MyriadGP::handleDegrade(MyriadGpDegrade* degrade, nodeid_t from) {
    if (!degradeEnabled) return;
    uint32_t R = degrade->getRing_id();
    uint32_t seq = degrade->getSeq_no(), beacon = degrade->getBeacon_no();
    if (!rib.applyDegrade(R, seq, beacon)) return;
    floodDegrade(R, seq, beacon, from);   // skip-sender
    MLOG_N_DEBUG(from) << "degrade ring='" << R << "' seq=" << seq << "\n";
}

void MyriadGP::reset() {
    Enter_Method_Silent();   // invoked cross-module from NetworkManager; we reschedule below
    for (auto& [nid, nbr] : neighbors) {
        if (nbr.deadTimer) {
            cancelAndDelete(nbr.deadTimer);
            nbr.deadTimer = nullptr;
        }
    }
    neighbors.clear();
    for (auto& [rid, m] : memberships) {
        cancelRingReliable(m.collectInflight);
        for (auto& [chain, fl] : m.updateInflights)
            cancelRingReliable(fl);
        m.updateInflights.clear();
        for (auto& [seq, fl] : m.batchInflights)
            cancelRingReliable(fl);
        m.batchInflights.clear();
        if (m.refreshTimer) {
            cancelAndDelete(m.refreshTimer);
            m.refreshTimer = nullptr;
        }
        if (m.watchdogTimer) {
            cancelAndDelete(m.watchdogTimer);
            m.watchdogTimer = nullptr;
        }
    }
    memberships.clear();
    rib.clear();
    pendingTransitions.clear();
    if (holdTimer) { cancelAndDelete(holdTimer); holdTimer = nullptr; }
    if (snapshotTimer) { cancelAndDelete(snapshotTimer); snapshotTimer = nullptr; }
    cancelHelloTimer();

    rebooted = true;   // stateless from here on means recovery, not formation

    scheduleHello(helloInterval);
    if (snapshotInterval > 0 && !par("ribDumpFile").stdstringValue().empty())
        startSnapshot();
}

void MyriadGP::dumpRibTo(std::ostream& f, const std::string& tprefix, bool stats) {
    rib.dumpTo(f, tprefix, stats, (int)myId);
}

void MyriadGP::finish() {
    recordSendScalars();
    recordScalar("batch_entries_total", batchEntriesTotal);
    recordScalar("batch_max_entries", batchMaxEntries);
    recordScalar("batches_created", batchesCreated);

    // Test hook (test_rib_batching.py). With periodic snapshots on, emit a final
    // t=<simTime> snapshot so the end state is captured
    std::string dumpFile = par("ribDumpFile").stdstringValue();
    if (!dumpFile.empty()) {
        std::ofstream f(dumpFile, std::ios::app);
        dumpRibTo(f, snapshotInterval > 0 ? "t=" + simTime().str() + " " : "", false);
    }
}

} // namespace routingsim
