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

HelloNeighbor& MyriadGP::findOrCreateHello(nodeid_t nid) {
    GpNeighbor& nbr = neighbors[nid];
    nbr.id = nid;
    return nbr;
}

HelloNeighbor* MyriadGP::findHello(nodeid_t nid) {
    auto it = neighbors.find(nid);
    return it == neighbors.end() ? nullptr : &it->second;
}

const HelloNeighbor* MyriadGP::findHello(nodeid_t nid) const {
    auto it = neighbors.find(nid);
    return it == neighbors.end() ? nullptr : &it->second;
}

void MyriadGP::forEachHello(std::function<void(HelloNeighbor&)> fn) {
    for (auto& [nid, nbr] : neighbors) fn(nbr);
}

void MyriadGP::forEachHello(std::function<void(const HelloNeighbor&)> fn) const {
    for (const auto& [nid, nbr] : neighbors) fn(nbr);
}

void MyriadGP::onTwoWayReached(HelloNeighbor& nbr) {
    nbr.state = NSTATE_FULL;
    MLOG_N_INFO(nbr.id) << "going FULL (path-vector adjacency up)\n";
    // TODO: send initial path-vector advertisement
}

void MyriadGP::onNeighborDead(HelloNeighbor& nbr, bool /*wasFull*/) {
    // TODO: withdraw paths learned from this neighbor and retract any best-paths that depended on it.
}

void MyriadGP::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    auto header = std::unique_ptr<MyriadGpHeader>(check_and_cast<MyriadGpHeader*>(pkt->decapsulate()));
    nodeid_t from = header->getRouter_id_src();
    switch (header->getType()) {
        case MYRIAD_HELLO:
            handleHello(check_and_cast<MyriadHello*>(header.get()), from);
            break;
        // TODO: path-vector update / withdraw / ack PDUs.
        default:
            MLOG_WARN() << "received unknown packet type: " << header->getType() << "\n";
    }
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
