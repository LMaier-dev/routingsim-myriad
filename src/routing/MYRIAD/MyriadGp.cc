#include "routing/MYRIAD/MyriadGp.h"

namespace routingsim {

Define_Module(MyriadGP);

void MyriadGP::onInitialize() {
    MyriadHelloProto::onInitialize();
    packetType = MYRIADGP_PACKET_TYPE;

    node().bind(MYRIADGP_PACKET_TYPE, this);

    MLOG_INFO() << "MyriadGP up\n";

    // Periodically send hellos to all neighbors. The hello / dead / two-way
    scheduleHello(helloInterval);
}

void MyriadGP::onTwoWayReached(GpNeighbor& nbr) {
    nbr.state = NSTATE_FULL;
    MLOG_N_INFO(nbr.id) << "Starting Collect phase\n";
    // TODO: collect phase now send Collect to this neighbor
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
