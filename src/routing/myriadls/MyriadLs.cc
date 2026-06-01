#include "routing/myriadls/MyriadLs.h"

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    MyriadHelper::onInitialize();
    packetType = MYRIADLS_PACKET_TYPE;
    helloInterval = par("helloInterval");
    deadInterval = par("deadInterval");
    rxmtInterval = par("rxmtInterval");

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
    }
    neighbors.clear();
    lsdb.clear();

    if (helloTimer) {
        cancelAndDelete(helloTimer);
        helloTimer = nullptr;
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
            nbr.haveMore = true; // for testing
            sendDBD(from, /*init=*/true, nbr.amMaster, /*more=*/true, nbr.ddSeq);
            break;
        }
    }
}

void MyriadLS::handleDBD(MyriadDBD* dbd, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];    
    if (myId > from) { // master
        if (dbd->getInit()) { // Phase 1 master negotioation
            nbr.amMaster = true;
            nbr.ddSeq = nbr.ddSeq + 1;
            // TODO build pending DBD list instead of using haveMore
            // Phase 2 intiated by master
            cancelReliable(nbr.pendingDbd);
            sendDBD(from, /*init=*/false, nbr.amMaster, /*more=*/true, nbr.ddSeq);
        } else {
            if (!dbd->getMaster() && dbd->getSeq_no() == nbr.ddSeq && nbr.state < NSTATE_FULL) {
                cancelReliable(nbr.pendingDbd);
                MLOG_N_DEBUG(from) << "received expected DBD from slave, seq=" << dbd->getSeq_no() << "\n";
                nbr.ddSeq++; // master increments before response
                sendDBD(from, /*init=*/false, nbr.amMaster, /*more=*/nbr.haveMore, nbr.ddSeq);
                nbr.haveMore = false;
                if (!dbd->getMore()) {
                    MLOG_N_INFO(from) << "slave has no more DBDs, going FULL\n";
                    nbr.state = NSTATE_FULL;
                    cancelReliable(nbr.pendingDbd);
                }
            } else if (!dbd->getMaster() && nbr.state == NSTATE_FULL) {
                MLOG_N_DEBUG(from) << "received additional DBD from slave after FULL, seq=" << dbd->getSeq_no() << "\n";
            } else {
                MLOG_N_WARN(from) << "received unexpected DBD from slave\n";
            }
        }
    } else { // slave
        if (dbd->getInit()) { // Phase 1 master negotioation
            nbr.amMaster = false;
            nbr.ddSeq = dbd->getSeq_no() + 1;
            // slave does not retransmit
            cancelReliable(nbr.pendingDbd);
            // TODO build pending DBD list
        } else {
            // slaves never retransmit (cancel just in case, but it should be null)
            cancelReliable(nbr.pendingDbd);
            // Phase 2 response by slave
            if (dbd->getMaster() && dbd->getSeq_no() == nbr.ddSeq && nbr.state < NSTATE_FULL) {
                MLOG_N_DEBUG(from) << "received expected DBD from master, seq=" << dbd->getSeq_no() << "\n";
                sendDBD(from, /*init=*/false, nbr.amMaster, /*more=*/nbr.haveMore, nbr.ddSeq);
                nbr.haveMore = false;
                if (!dbd->getMore()) {
                    MLOG_N_INFO(from) << "master has no more DBDs, going FULL\n";
                    nbr.state = NSTATE_FULL;
                }
                nbr.ddSeq++; // slave increments after response
            } else {
                MLOG_N_WARN(from) << "received unexpected DBD from master\n";
            }
        }
    }
}

void MyriadLS::sendDBD(nodeid_t to, bool init, bool master, bool more, uint16_t seq_no) {
    auto* dbd = new MyriadDBD("Myriad_DBD");
    dbd->setRouter_id_src(myId);
    dbd->setInit(init);
    dbd->setMaster(master);
    dbd->setMore(more);
    dbd->setSeq_no(seq_no);
    // TODO ls_dbds left empty for now

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
    nbr.amMaster = false;
    nbr.ddSeq = 0;
}

} // namespace routingsim

