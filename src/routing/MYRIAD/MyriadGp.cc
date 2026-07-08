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
    // TODO: budget timer — cancels inflight after 3 retries → DEGRADE.

    MLOG_N_INFO(to) << "originate COLLECT ring='" << (char)ring
        << "' seq=" << rs.inflight.seq_no << "\n";
}

void MyriadGP::onNeighborDead(GpNeighbor& nbr, bool /*wasFull*/) {
    MLOG_N_INFO(nbr.id) << "Send Degrade now\n";
    // TODO: Degrade this ring
}

void MyriadGP::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    auto inner = std::unique_ptr<cPacket>(pkt->decapsulate());

    // Hello packets come from MyriadHello so dont go through gpheader path
    if (auto* hello = dynamic_cast<MyriadHello*>(inner.get())) {
        handleHello(hello, hello->getRouter_id_src());
        return;
    }

    auto* header = check_and_cast<MyriadGpHeader*>(inner.get());
    nodeid_t from = header->getRouter_id_src();
    switch (header->getType()) {
        case MYRIADGP_COLLECT:
            handleCollect(check_and_cast<MyriadGpCollect*>(header), from);
            break;
        case MYRIADGP_UPDATE:
            handleUpdate(check_and_cast<MyriadGpUpdate*>(header), from);
            break;
        case MYRIADGP_DEGRADE:
            handleDegrade(check_and_cast<MyriadGpDegrade*>(header), from);
            break;
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }
}

void MyriadGP::handleCollect(MyriadGpCollect* collect, nodeid_t from) {
    GpNeighbor& nbr = neighbors[from];
    MLOG_N_INFO(from) << "Received Collect seq=" << collect->getSeq_no()
                      << " shared rings=[";
    for (size_t i = 0; i < nbr.sharedRings.size(); ++i)
        EV << (i ? "," : "") << (char)nbr.sharedRings[i];
    EV << "]\n";
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
    cancelHelloTimer();
}

void MyriadGP::finish() {
    // TODO: dump path-vector table.
}

} // namespace routingsim
