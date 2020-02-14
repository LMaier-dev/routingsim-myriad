//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ROUTINGSIM_R2Kademlia_H_
#define __ROUTINGSIM_R2Kademlia_H_

#include <omnetpp.h>
#include <unordered_map>

#include "foundation.h"
#include "Protocol.h"
#include "StatisticSample.h"
#include "KadPackets_m.h"
#include "NeighborTable.h"
#include "KadRoutingTable.h"
#include "FwdTable.h"
#include "RPC.h"

namespace routingsim {

// packet types
using KadDataPacket_p = std::unique_ptr<KadDataPacket>;
using KadPacket_p = std::unique_ptr<KadPacket>;
using KadAnswer_p = std::unique_ptr<KadAnswer>;
using KadHello_p = std::unique_ptr<KadHello>;
using KadError_p = std::unique_ptr<KadError>;

using UpdateList_t = std::list< std::pair<KadContact, KadRouteUpdateAction> >;

enum reasoncode : uint8_t {
	unspecified = 0,
        hopLimitExceeded = 1,	
	nextHopUnknown = 2,
	travelledDisabledLink = 3,
	wrongNextHop= 4,
	routeFailureDeadEnd= 5,
	secondChanceFailed = 6,
	missingNeighbor = 7,
	missingPID = 8,
	unhandledType= 128
};



/**
 * Implementation of R2Kademlia.
 * Routing with R/Kademlia...
 * TODO: sources
 * @author Jakob Thomas
 */
class R2Kademlia : public RPCProtocol, public KadPolicy, public StatisticSample
{
  protected:
	/** @name Protocol interface */
	//@{
	/// called at initialization time of the Protocol
	void onInitialize() override;
	/// called when a new node is connected
	void onConnect(nodeid_t nid) override;
	/// called when a node is disconnected
	void onDisconnect(nodeid_t nid) override;
	/// on message
	void onMessage(NodePkt_p pkt) override;
	/// on self message
	void onSelfMessage(cMessage_p msg) override;
	/// reset state of Protocol to initial state (Node Failure)
	void reset() override;
	//@}

	/** @name packet handling */
	//@{
	/// adds the neighbor to the neighbor table
	virtual void handlePacket(KadHello_p packet);
	/// responds to probe request
	virtual void handlePacket(std::unique_ptr<KadProbeReq> packet);
	virtual void handlePacket(std::unique_ptr<KadPathSetupReq> packet);
	virtual void handlePacket(std::unique_ptr<KadPathTeardownReq> packet);
	virtual void handlePacket(std::unique_ptr<KadPNDiscReq> packet);
	virtual void handlePacket(std::unique_ptr<KadQueryRouteReq> packet);
	virtual void handlePacket(std::unique_ptr<KadUpdateRouteReq> packet);
	virtual void handlePacket(std::unique_ptr<KadFindNodeReq> packet);
	virtual void handlePacket(KadAnswer_p packet);
	virtual void handleDataPacket(NodePkt_p packet);
	//@}

	/** @name sending packets out*/
	//@{
	/**
	 * forwards a packet to a neighbor based on the source route in the packet.
	 * uses the given NodePkt for encapsulation.
	 * This does not make any routing decision.
	 */
	void forwardPacket(KadPacket_p packet, NodePkt_p nodePkt);
	/// forwards a data packet
	void forwardDataPacket(KadDataPacket_p packet, NodePkt_p nodePkt);
	/// forwards a data packet based on PathID present in the packet
	bool forwardDataPacketToNextHop(KadDataPacket_p& packet, NodePkt_p& nodePkt);
	
	/// sends a received packet again after the source route was manipulated
	void resendPacket(KadPacket_p packet);
	/// routes a packet to a neighbor based on the source route in the packet
	void sendPacket(KadPacket_p packet) override;
	/// routes a packet to a neighbor based on the  in the packet
	void sendPacket(KadDataPacket_p packet);
	/// chooses the contact to forward a request to according to the overlay metric.
	const KadContact& chooseNextHop(const KadContactList&, const OverlayKey& target, const OverlayKey& source);
	/// sends a probe request to the given contact
	void sendProbe(KadContact_p contact);
	/// sends a path setup request to the given contact
	void sendPathSetup(KadContact_p contact);
	/// sends a "get physical neighbors" call to the contact
	void sendGetPhysicalNeighbors(KadContact_p contact);
	/// sends a "get overlay neighbors" call to the contact
	void sendGetOverlayNeighbors(KadContact_p contact);
	/// sends a PNDisc Request to the Physical Neighbor
	void sendDiscoveryReq(KadContact_p contact);
	/// sends a find node call to find this node (rediscovery may use a not via failed link)
	void sendFindNode(const OverlayKey& key, bool exact=false, const OverlayKey* notvia_unreachable_node= nullptr, const OverlayKey* notvia_failednode_neighbor= nullptr);
	/// sends a data packet
	void sendData(const OverlayKey& key);
	//@}

	/** @name policy foo */
	//@{
	void onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt);
	void onMissingNeighbor(KadDataPacket_p packet, NodePkt_p pkt);
	/** Called when an error packet is encountered, updates routing information */
	void onErrorPacket(KadError& error);
	//@}
	virtual void sampleStatistics (Statistics &stats) override;
public:
        R2Kademlia() : routingTable(neighborTable) {};
	/** @name policy functions called by routing table */
	//@{
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) override;
	/** called when a new contact is created in the routing table*/
	virtual void onNewContact(KadContact_p contact) override;
	/** called when a contact is going to be deleted from routing table*/
	virtual void onContactRemoval(KadContact_p contact) override;

	/** Called when a route got improved */
	virtual void onImprovement(const KadContact& contact) override;
	//@}


	uint getHopLimitDefault() const { return hopLimit; };

	/** @name statistics functions used by statistics module */
	//@{
	const KadRoutingTable& getRoutingTable() const { return routingTable; }
	const OverlayKey& getOverlayKey() const { return myOverlayKey; }
	uint getForwardedPackets() const { return forwardedPackets; }
	uint getDroppedPackets() const { return droppedPackets; }
	uint getSentPackets() const { return sentPackets; }
	uint getReceivedPackets() const { return receivedPackets; }

 	uint getForwardedPathSetup() const { return forwardedPathSetup; }
	uint getSentPathSetup() const { return sentPathSetup; }
	uint getReceivedPathSetup() const { return receivedPathSetup; }
	uint getReceivedPathTeardown() const { return receivedPathTeardown; }
	
 	uint getForwardedDataPackets() const { return forwardedDataPackets; }
	uint getDroppedDataPackets() const { return droppedDataPackets; }
	uint getSentDataPackets() const { return sentDataPackets; }
	uint getReceivedDataPackets() const { return receivedDataPackets; }
	void recordScalars();
	void recordRoutingTable();

	//@}

private:
	/** @name message generation */
	//@{
	/// crafts a hello packet and encapsulates it in a NodePkt
	NodePkt_p makeHelloPacket();
	/// crafts a packet of type T, sets the route to the given contact.
	template<typename T>
	std::unique_ptr<T> makePacket(const KadContact* contact, const char *name=nullptr);
	template<typename T>
	std::unique_ptr<T> makePacket(const KadContact& contact, const char *name=nullptr){
		return makePacket<T>(&contact, name);
	};
	/// crafts a data packet, sets the DestNodeID to first argument and also PathID according to the given viacontact.
	template<typename T>
	std::unique_ptr<T>makeDataPacket(const OverlayKey& dstNodeID, const KadContact* viacontact, const char *name);
	/// crafts a packet of type T, sets the route and nonce according to original
	template<typename T>
	std::unique_ptr<T> makeAnswer(KadPacket& original);
	/// saves the contact list to the given packet.
	void fillRTableRsp(KadRTableRsp& packet, const KadContactList& contacts, bool includePaths=true);
	/// reads the contacts from the packet and adds the paths to the routing table
	void readRTableRsp(std::unique_ptr<KadRTableRsp> packet);
	//@}

	/** @name timer functions */
	//@{
	/// adds a timer to the timer vectors, sends a timer packet
	void addTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback);
	void doProbe(short timerId);
	void doProbeSelf(short timerId);
	void doDataProbe(short timerId);
	void doReachabilityTest(short timerId);
	//@}

	/** @name private helpers */
	//@{
	/// does statistics on received packet
	void countReceived(KadPacket& packet);
	/// does statistics on dropped packet
	void countDropped(KadPacket& packet, const OverlayKey& target, uint8_t reason);
	/// does statistics on received data packet
	void countReceived(KadDataPacket& packet);
	/// does statistics on dropped data packet
	void countDropped(KadDataPacket& packet, const OverlayKey& target, uint8_t reason);
	/// try a shortcut route and modify source route of the packet
	void applyShortcut(KadPacket& packet);
	/// try to install path from response packet
	void checkForPathInstallation(KadPacket& packet);

	/**
	 * sends an error packet to the sender of the given packet
	 * the original packet is encapsulated on the error packet
	 * @errorType the error type for the error packet
	 */
	void returnToSender(uint8_t errorType, KadPacket_p packet);
	void returnToSender(uint8_t errorType, KadDataPacket_p packet);
	/**
	 * Tries to cast packet to type T.
	 * If this succeeds, the method handlePacket(T) is called, the given pointer
	 * is invalidated and we return true.
	 * If the cast fails, nothing happens and we return false.
	 */
	template<typename T>
	bool handleIfFitting(KadPacket_p& packet);
	kad_path concatViaPath(const kad_path& viaPath, const OverlayKey& viaKey, const kad_path& path);
	/// adds contact and not via link for rediscovery queue
	void scheduleRediscovery(const OverlayKey& contactkey, const OverlayKey& notvia_unreachable_node, const OverlayKey& notvia_failednode_neighbor);
	void rediscoverContacts();
	/// adds routing table updates for contacts to an update queue
	void scheduleUpdate(const KadContact& affectedcontact, KadRouteUpdateAction action);
	/// send Update closer towards target contacts
	void propagateUpdateReq(std::unique_ptr<KadUpdateRouteReq> packet);
	/// sends all queued updates
	void sendUpdates();
	/// schedule a PathSetup
	void schedulePathSetup(const OverlayKey& contactkey);
	/// send all queue (valid) path setups
	void sendScheduledPathSetups();

	//@}

	/** @name callback functions for RPC calls */
	//@{
	bool handleProbeResponse(KadPacket_p answer, RPCState& rpcState);
	bool handlePathSetupResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState);
	bool handlePNDiscResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleFindNodeResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleRPCTimeOut(KadPacket_p answer, RPCState& rpcState);
	//@}
private:
	/// the overlay key of this node
	OverlayKey myOverlayKey;
	/// Table of direct physical neighbors
	NeighborTable neighborTable;
	/// Kademlia-like Routing Table containing Paths to contacts
	KadRoutingTable routingTable;

	/// Forwarding Table according to node IDs
	/// fwtabwdTableNodeID fwdTableNodeID;

	/// Forwarding Table according to Path IDs
	FwdTablePathID fwdTablePathID;
	
	/// packet type used for Node::bind()
	static const int KAD_CTRL_PACKET_TYPE = 5;
	static const int KAD_DATA_PACKET_TYPE = 6;

	/** @name config, set by parameters*/
	//@{
	/// number of contacts returned by find node response, called "k" in research papers
	size_type findNodeNumber;
	/// discoveryRadius= radius of physical neighborhood discovery (discover n-hop neighborhood)
	/// radius of 1 means only to discover physical neighbors
	/// radius of two means to ask the physical neighbors for their physical neighbors, too
	uint8_t discoveryRadius;
	/// activates proximity routing
	bool proximityRouting;
	// proximityNeighborSelection is in KadRoutingTable
	///
	uint hopLimit;
	/// if activated the node collects path information from passing packets
	bool pathSnooping;
	bool sendDataPackets;
	bool usePathIDs;
	uint8_t probeStrategy;
	uint8_t rediscoveryFanOutFactor= 3;
	simtime_t rediscoveryWaitTime;
	simtime_t pathSetupWaitTime;
	simtime_t updateWaitTime;
	bool detailedRoutingTableStats;
	//@}

	// a list that is used to send rediscover messages
	std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	// selfMessage
	bool updateScheduled= false;

	// a list that is used to send rediscover messages
	std::list<std::unique_ptr<const KadContact>> rediscoverQueue;

	// a list that is used to send pathsetup messages
	std::list<OverlayKey> pathSetupQueue;

	// selfMessage
	bool pathSetupScheduled= false;

	// selfMessage
	bool rediscoveryScheduled= false;
	
	/** @name statistics */
	//@{
	uint forwardedPackets = 0;
	uint droppedPackets = 0;
	uint sentPackets = 0;
	uint receivedPackets = 0;
	uint forwardedDataPackets = 0;
	uint droppedDataPackets = 0;
	uint sentDataPackets = 0;
	uint receivedDataPackets = 0;
	uint sentRediscovery = 0;
	uint succeededRediscovery = 0;
	uint droppedRediscovery = 0;
	uint forwardedRediscovery = 0;
	uint inconsistentPathHit = 0;
	uint sentRouteUpdates = 0;
	uint receivedRouteUpdates = 0;
	uint sentPathSetup = 0;
	uint receivedPathSetup = 0;
	uint forwardedPathSetup = 0;
	uint receivedPathTeardown = 0;
	//@}

	unsigned long laterpaths = 0;
	unsigned long shortenedlaterpaths = 0;
	unsigned long viablePIDpaths = 0;
	
	/** @name timer things */
	//@{
	std::vector<simtime_t> timerDuration;
	std::vector<std::function<void(short)>> timerCallback;
	std::vector<cMessage *> timerMsg;
	//@}
};

// template functions

template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(const KadContact* contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->setHopLimit(hopLimit);
	packet->appendRoute(contact);
	return packet;
}

template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(KadPacket& original) {
	auto answer = std::make_unique<T>();
	answer->setName((std::string(original.getName()) + std::string("Answer")).c_str());
	if (routingTable.usePathSimplification())
		answer->setShortened(answer->setSimplifiedReturnPathFrom(original));
	else
	{
		answer->setReturnPathFrom(original);
		answer->setShortened(false);
	}

	ASSERT(answer->getSource() == myOverlayKey);
	answer->setNonce(original.getNonce());
	return answer;
}

/**
 * Cast a unique_ptr for a KadPacket to another unique_ptr for a different packet.
 */
template<typename T> std::unique_ptr<T> safe_packet_cast(KadPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	// this will throw cRuntimeError if the cast fails
	return std::unique_ptr<T>(check_and_cast<T*>(packet.release()));
}

} //namespace

#endif // __ROUTINGSIM_R2Kademlia_H_
