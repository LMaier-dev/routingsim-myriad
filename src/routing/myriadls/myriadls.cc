#include "routing/myriadls/myriadls.h"

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    myId = node().getNodeId();
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

}

void MyriadLS::onConnect(nodeid_t nid) {}
void MyriadLS::onDisconnect(nodeid_t nid) {}
void MyriadLS::onMessage(NodePkt_p pkt) {}
void MyriadLS::reset() {}

} // namespace routingsim

