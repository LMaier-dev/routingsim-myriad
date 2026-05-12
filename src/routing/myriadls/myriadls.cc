#include "routing/myriadls/myriadls.h"

namespace routingsim {

Define_Module(MyriadLS);

void MyriadLS::onInitialize() {
    EV << "Hello, World! I'm node " << node().getNodeId() << ".\n";
}

void MyriadLS::onConnect(nodeid_t nid) {}
void MyriadLS::onDisconnect(nodeid_t nid) {}
void MyriadLS::onMessage(NodePkt_p pkt) {}
void MyriadLS::reset() {}

} // namespace routingsim

