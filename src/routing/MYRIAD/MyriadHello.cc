#include "routing/MYRIAD/MyriadHello.h"

namespace routingsim {

void MyriadHelloProto::onInitialize() {
    MyriadHelper::onInitialize();
    helloInterval = par("helloInterval");
    deadInterval = par("deadInterval");
    rxmtInterval = par("rxmtInterval");
}

NeighborState MyriadHelloProto::getNeighborState(nodeid_t nbr) const {
    const HelloNeighbor* h = findHello(nbr);
    return h ? h->state : NSTATE_DOWN;
}

void MyriadHelloProto::onConnect(nodeid_t nid) {
    HelloNeighbor& nbr = findOrCreateHello(nid);
    nbr.id = nid;
    nbr.state = NSTATE_DOWN;
}

void MyriadHelloProto::onDisconnect(nodeid_t nid) {
    handleDead(nid);
}

void MyriadHelloProto::scheduleHello(simtime_t delay) {
    helloTimer = addOneShotTimer(delay, [this](short) {
        helloTimer = nullptr; // framework deletes this TimerMsg after we return
        sendHello();          // reschedules and assigns the new helloTimer
        return false;
    });
}

void MyriadHelloProto::sendHello() {
    auto hello = std::make_unique<MyriadHello>("Myriad_Hello");
    hello->setRouter_id_src(myId);
    // Currently not read on the receiving end, but set it anyway for completeness
    hello->setHelloInterval(helloInterval);
    hello->setDeadInterval(deadInterval);

    // Only advertise neighbors we've actually heard a hello from
    std::vector<nodeid_t> heard;
    forEachHello([&](const HelloNeighbor& nbr) {
        if (nbr.state >= NSTATE_INIT) heard.push_back(nbr.id);
    });
    hello->setKnownNeighborsArraySize(heard.size());
    for (size_t i = 0; i < heard.size(); ++i) {
        hello->setKnownNeighbors(i, heard[i]);
    }

    // TODO set hello->setByteLength
    forEachHello([&](const HelloNeighbor& nbr) {
        auto np = std::make_unique<NodePkt>("Myriad_Hello_Pkt");
        np->setType(packetType);
        np->encapsulate(hello->dup());
        sendToNode(nbr.id, std::move(np));
    });
    scheduleHello(helloInterval);
}

void MyriadHelloProto::handleHello(MyriadHello* hello, nodeid_t from) {
    HelloNeighbor& nbr = findOrCreateHello(from);
    nbr.id = from;
    resetDead(from);

    // HelloReceived: DOWN -> INIT (can subsequently go into TWOWAY if my id is in their hello)
    if (nbr.state == NSTATE_DOWN) {
        nbr.state = NSTATE_INIT;
        MLOG_N_INFO(from) << "received first hello (INIT)\n";
    }

    if (nbr.state >= NSTATE_TWOWAY) return; // already past two-way

    // 2-WayReceived: INIT -> TWOWAY
    int size = hello->getKnownNeighborsArraySize();
    for (int i = 0; i < size; i++) {
        if (hello->getKnownNeighbors(i) == myId) {
            nbr.state = NSTATE_TWOWAY;
            MLOG_N_INFO(from) << "now in TWO-WAY state\n";
            onTwoWayReached(nbr);
            break;
        }
    }
}

void MyriadHelloProto::resetDead(nodeid_t nid) {
    HelloNeighbor* nbr = findHello(nid);
    if (!nbr) return;
    // nullptr is accepted (i.e. on first hello)
    cancelAndDelete(nbr->deadTimer);
    nbr->deadTimer = addOneShotTimer(deadInterval, [this, nid](short) {
        handleDead(nid);
        return false;
    });
}

void MyriadHelloProto::handleDead(nodeid_t nid) {
    HelloNeighbor* nbr = findHello(nid);
    if (!nbr) return;
    bool wasFull = (nbr->state == NSTATE_FULL);

    MLOG_N_WARN(nid) << "neighbor timed out (DEAD)\n";
    nbr->deadTimer = nullptr;
    nbr->state = NSTATE_DOWN;
    onNeighborDead(*nbr, wasFull);
}

void MyriadHelloProto::cancelHelloTimer() {
    if (helloTimer) {
        cancelAndDelete(helloTimer);
        helloTimer = nullptr;
    }
}

} // namespace routingsim
