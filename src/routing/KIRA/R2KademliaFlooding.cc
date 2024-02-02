#include "KIRA.h"
#include "KadPacket_m.h"
#include <cstdint>

namespace routingsim {
uint32_t R2Kademlia::replicateFloodPacket(std::unique_ptr<KadFloodPacket>& packet) {


    const auto floodPrefixLength(packet->getFloodingPrefixLength());

    const auto replication_contacts = routingTable.getReplicationContacts(floodPrefixLength);
    for (auto& repl_contact: replication_contacts) {
	    auto floodPacket = makeDataPacket<KadFloodPacket>(std::get<0>(repl_contact)->getKey(),std::get<0>(repl_contact), "KadFloodPacket");
        floodPacket->setFloodingPrefixLength(std::get<1>(repl_contact));
        // rewrite the original source 
	    floodPacket->setSrcNodeID(packet->getSrcNodeID());
	    floodPacket->setHopLimit(packet->getHopLimit());
		sendPacket(std::move(floodPacket));
    }
    return replication_contacts.size();
}

void R2Kademlia::handlePacket(std::unique_ptr<KadFloodPacket> packet) {
    
    // check if we need to replicate
    auto replications = replicateFloodPacket(packet);
    countReceived(*packet, replications);
    EV_TRACE << "Got KadFloodPacket" << std::endl;
}

void R2Kademlia::countReceived(KadFloodPacket& packet, uint32_t replications) {
    floodPacketReplications += replications;
    floodPacketMaxReplications = std::max(floodPacketMaxReplications, replications);
	floodPacketsReceived++;
}


bool R2Kademlia::sendFloodPacket(short timerID) {

    EV_TRACE << "Send KadFloodPacket" << std::endl;
    // create a flood packet and replicate directly
    const auto floodPrefixLength(0);

    const auto replication_contacts = routingTable.getReplicationContacts(floodPrefixLength);

    for (auto& repl_contact: replication_contacts) {
	    auto floodPacket = makeDataPacket<KadFloodPacket>(std::get<0>(repl_contact)->getKey(),std::get<0>(repl_contact), "KadFloodPacket");
        floodPacket->setFloodingPrefixLength(std::get<1>(repl_contact));
		sendPacket(std::move(floodPacket));
    }
    return false;

}

} // end namespace

