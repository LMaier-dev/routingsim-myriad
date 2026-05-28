#include "routing/myriadls/MyriadLs.h"

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    myId = node().getNodeId();
    helloInterval = par("helloInterval");
    deadInterval = par("deadInterval");

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
            break;
        }
    }
}

void MyriadLS::handleDBD(MyriadDBD* dbd, nodeid_t from) {
    NeighborEntry& nbr = neighbors[from];
    if (nbr.state == NSTATE_TWOWAY) return; // already in two-way, no state change
    EV << myId << " received first DBD from " << from << "\n";
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
}

} // namespace routingsim

