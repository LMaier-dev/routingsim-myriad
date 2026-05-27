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
    EV << "Node " << myId << " connected to " << nid << "\n";

    NeighborEntry& nbr = neighbors[nid];
    nbr.id = nid;
    nbr.state = NSTATE_INIT;
}

void MyriadLS::onDisconnect(nodeid_t nid) {}

void MyriadLS::onMessage(NodePkt_p pkt) {
    myId = node().getNodeId();
    auto header = std::unique_ptr<MyriadHeader>(check_and_cast<MyriadHeader*>(pkt->decapsulate()));
    nodeid_t from = header->getRouter_id_src();    
    switch (header->getType()) {
        case MYRIAD_HELLO: {
            auto hello = check_and_cast<MyriadHello*>(header.get());
            EV << myId << " received hello from " << from << "\n";
            break;
        }
        default:
            EV << myId << " received unknown packet type: " << header->getType() << "\n";
    }

}

void MyriadLS::onSelfMessage(cMessage_p msg) {
      auto* timer = dynamic_cast<TimerMsg*>(msg.get());
      if (timer) {
          auto* cb = static_cast<TimerMsg::cbfunc_t*>(timer->getContextPointer());
          if ((*cb)(timer->getKind())) {
              delete timer;
          }
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
    // EV << "Hello There\n";
    auto hello = std::make_unique<MyriadHello>("Myriad_Hello");
    hello->setRouter_id_src(myId);
    hello->setHelloInterval(helloInterval);
    hello->setDeadInterval(deadInterval);
    hello->setKnownNeighborsArraySize(neighbors.size());
    int idx = 0;
    for (auto& [nid, nbr] : neighbors) {
        hello->setKnownNeighbors(idx++, nid);
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



} // namespace routingsim

