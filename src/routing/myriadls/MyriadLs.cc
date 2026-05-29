#include "routing/myriadls/MyriadLs.h"

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    myId = node().getNodeId();
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

    EV << "Node " << node().getNodeId() << " rings: " << rings << "\n";

    // periodically send hellos to all neighbors
    scheduleHello(helloInterval);

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
            EV << myId << " received unknown packet type: " << header->getType() << "\n";
    }

}

void MyriadLS::onSelfMessage(cMessage_p msg) {
      auto* timer = dynamic_cast<TimerMsg*>(msg.get());
      if (timer) {
          auto* cb = static_cast<TimerMsg::cbfunc_t*>(timer->getContextPointer());
          (*cb)(timer->getKind());
      }
  }


void MyriadLS::reset() {}


void MyriadLS::scheduleHello(simtime_t delay) {
    addOneShotTimer(delay, [this](short) {
        sendHello();
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
        EV << myId << " received first hello from " << from << " (INIT)\n";
    }

    if (nbr.state >= NSTATE_TWOWAY) return; // already past two-way

    // 2-WayReceived: INIT -> TWOWAY
    int size = hello->getKnownNeighborsArraySize();
    for (size_t i = 0; i < size; i++) {
        if (hello->getKnownNeighbors(i) == myId) {
            nbr.state = NSTATE_TWOWAY;
            EV << myId << " and " << from << " are now in TWO-WAY state\n";
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
                EV << myId << " received expected DBD from slave " << from << ", seq NO " << dbd->getSeq_no() << "\n";
                nbr.ddSeq++; // master increments before response
                sendDBD(from, /*init=*/false, nbr.amMaster, /*more=*/nbr.haveMore, nbr.ddSeq);
                nbr.haveMore = false;
                if (!dbd->getMore()) {
                    EV << myId << " slave " << from << " has no more DBDs to send, neighbor is now FULL\n";
                    nbr.state = NSTATE_FULL;
                    cancelReliable(nbr.pendingDbd);
                }
            } else if (!dbd->getMaster() && nbr.state == NSTATE_FULL) {
                EV << myId << " received additional DBD from slave " << from << " after FULL, seq NO " << dbd->getSeq_no() << "\n";
            } else {
                EV << myId << " received unexpected DBD from slave " << from;
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
                EV << myId << " received expected DBD from master " << from << ", seq NO " << dbd->getSeq_no() << "\n";
                sendDBD(from, /*init=*/false, nbr.amMaster, /*more=*/nbr.haveMore, nbr.ddSeq);
                nbr.haveMore = false;
                if (!dbd->getMore()) {
                    EV << myId << " master " << from << " has no more DBDs to send, neighbor is now FULL\n";
                    nbr.state = NSTATE_FULL;
                }
                nbr.ddSeq++; // slave increments after response
            } else {
                EV << myId << " received unexpected DBD from master " << from;
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

PendingTx* MyriadLS::sendReliable(nodeid_t to, cPacket* pkt, simtime_t interval) {
    auto* h = new PendingTx{pkt, to, interval, nullptr};

    // First send: dup() so we keep the template for retransmits
    auto np = std::make_unique<NodePkt>(pkt->getName());
    np->setType(MYRIADLS_PACKET_TYPE);
    np->encapsulate(pkt->dup());
    sendToNode(to, std::move(np));

    h->timer = addOneShotTimer(interval, [this, h](short) {
        retxFire(h);
        return false;
    });
    return h;
}

void MyriadLS::retxFire(PendingTx* h) {
    auto np = std::make_unique<NodePkt>(h->pkt->getName());
    np->setType(MYRIADLS_PACKET_TYPE);
    np->encapsulate(h->pkt->dup());
    sendToNode(h->dest, std::move(np));

    h->timer = addOneShotTimer(h->interval, [this, h](short) {
        retxFire(h);
        return false;
    });
}

void MyriadLS::cancelReliable(PendingTx*& h) {
    if (!h) return;
    if (h->timer) cancelAndDelete(h->timer);
    delete h->pkt;
    delete h;
    h = nullptr;
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

    EV << myId << " neighbor " << nid << " timed out (DEAD)\n";
    nbr.deadTimer = nullptr;
    nbr.state = NSTATE_DOWN;
    cancelReliable(nbr.pendingDbd);
    nbr.amMaster = false;
    nbr.ddSeq = 0;
}

} // namespace routingsim

