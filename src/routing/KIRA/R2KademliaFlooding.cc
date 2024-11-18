#include "KIRA.h"
#include "KadPacket_m.h"
#include "KadContact.h"
#include "KadFloodObserver.h"
#include "KadPacket_m.h"
#include "KadPackets_m.h"
#include "NodePkt_m.h"
#include "OverlayKey128.h"
#include "Protocol.h"
#include <cstdint>
#include <memory>
#include <optional>

namespace routingsim {


bool R2Kademlia::sendFloodPacket(NodePkt_p pkt) {
    Enter_Method("sendFloodPacket(pkt)");
    EV_TRACE << "Send FloodPacket" << std::endl;
    // create a flood packet and replicate directly
    const auto payload_ = pkt.release();
    take(payload_);
    auto payload = std::shared_ptr<NodePkt>(payload_);
    return sendFloodPacket(std::nullopt, payload);
}

uint32_t R2Kademlia::replicateFloodPacket(KadFloodPacket& packet) {
    const auto floodPrefixLength(packet.getFloodingPrefixLength());

    const auto replication_contacts = routingTable.getReplicationContacts(floodPrefixLength, floodStrideFactor, floodCompleteBucketOpt);
    for (auto& repl_contact: replication_contacts) {
	    auto floodPacket = createReplicationPacket(packet, std::get<0>(repl_contact), std::get<1>(repl_contact));
		sendPacket(std::move(floodPacket));
    }
    return replication_contacts.size();
}

std::unique_ptr<KadFloodPacket> R2Kademlia::createReplicationPacket(KadFloodPacket& packet, KadContact_p contact, uint8_t floodPrefixLength) {
	    auto floodPacket = makeDataPacket<KadFloodPacket>(contact->getKey(), contact, "KadFloodPacket");
        floodPacket->setFloodingPrefixLength(floodPrefixLength);
        // rewrite the original source 
	    floodPacket->setSrcNodeID(packet.getSrcNodeID());
	    floodPacket->setHopLimit(packet.getHopLimit());
	    floodPacket->setOverlayHops(packet.getOverlayHops() + 1);
	    floodPacket->setNonce(packet.getNonce());
        floodPacket->setType(KAD_Flood);
        floodPacket->setPayload(packet.getPayload());

        return floodPacket;
}

std::unique_ptr<KadFloodPacket> R2Kademlia::createFloodPacket(KadContact_p contact, uint8_t floodPrefixLength, NodePkt_ps payload) {
	    auto floodPacket = makeDataPacket<KadFloodPacket>(contact->getKey(), contact, "KadFloodPacket");
        floodPacket->setFloodingPrefixLength(floodPrefixLength);
        floodPacket->setType(KAD_Flood);
        floodPacket->setPayload(payload);
        return floodPacket;
}

void R2Kademlia::handlePacket(std::unique_ptr<KadFloodPacket> packet) {
    
    EV_TRACE << "Got KadFloodPacket" << std::endl;
    // check if we need to replicate
    auto replications = replicateFloodPacket(*packet);
    countReceived(*packet, replications);

    //deliver the payload locally
    if (auto payload = packet->getPayload(); payload != nullptr) {
        EV_DEBUG << "Got Payload of FloodPacket" << std::endl;
        auto pkt = new NodePkt(*payload);
        sendLocal(pkt);
    }

    auto& floodObserver = getKadFloodObserver(); 
    const auto floodPrefixLength(packet->getFloodingPrefixLength());
    floodObserver.registerFloodReplication(packet->getNonce(), myOverlayKey, Node::getNode(this)->getNodeId(), floodPrefixLength, replications);
    floodObserver.registerFloodDelivered(packet->getNonce(), myOverlayKey, Node::getNode(this)->getNodeId(), *packet);

}

void R2Kademlia::handleFloodPacketDropped(const KadFloodPacket& packet) {
   EV_TRACE << "handleFloodPacketDropped" << std::endl;
   getKadFloodObserver().registerFloodDropped(packet.getNonce(), myOverlayKey, Node::getNode(this)->getNodeId(), packet);  
}

void R2Kademlia::handleFloodPacketForwarded(const KadFloodPacket& packet, hop_t taken_hop) {
   EV_TRACE << "handleFloodPacketForwarded" << std::endl;
   getKadFloodObserver().registerFloodForwarded(packet.getNonce(), myOverlayKey, Node::getNode(this)->getNodeId(), packet, taken_hop);  
}

void R2Kademlia::countReceived(KadFloodPacket& packet, uint32_t replications) {
    floodPacketReplications += replications;
    floodPacketMaxReplications = std::max(floodPacketMaxReplications, replications);
	floodPacketsReceived++;
}


bool R2Kademlia::sendFloodPacket(std::optional<hop_t> event, NodePkt_ps payload) {

    EV_TRACE << "Send KadFloodPacket" << std::endl;
    // create a flood packet and replicate directly
    auto& floodObserver = getKadFloodObserver(); 
    const auto floodId = floodObserver.registerFlood(myOverlayKey, Node::getNode(this)->getNodeId(), event);
    const auto replication_contacts = routingTable.getReplicationContacts(0, floodStrideFactor, floodCompleteBucketOpt);
    floodObserver.registerFloodReplication(floodId, myOverlayKey, Node::getNode(this)->getNodeId(), 0, replication_contacts.size());

    EV_TRACE << "sendFloodPacket() floodId:" << floodId << " Replications" << replication_contacts.size() << std::endl;

    for (auto& repl_contact: replication_contacts) {
	    auto floodPacket = createFloodPacket(std::get<0>(repl_contact),std::get<1>(repl_contact), payload);
        EV_TRACE << "Send KadFloodPacket" << floodId << " to " << floodPacket->getDstNodeID() << std::endl;
        floodPacket->setNonce(floodId);
        floodPacket->setPayload(payload);
		sendPacket(std::move(floodPacket));
    }
    return false;
}

} // end namespace

