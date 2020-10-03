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
#include <queue>

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

constexpr uint8_t OverlayKeyLength= 112;

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
	void forwardPacket(KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay= 0);
	/// forwards a data packet
	void forwardDataPacket(KadDataPacket_p packet, NodePkt_p nodePkt);
	/// forwards a data packet based on PathID present in the packet
	bool forwardDataPacketToNextHop(KadDataPacket_p& packet, NodePkt_p& nodePkt);
	
	/// sends a received packet again after the source route was manipulated
	void resendPacket(KadPacket_p packet);
	/// try to find alternative next overlay hop
	void reroutePacket(const OverlayKey& finaldest, KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay= 0);
	/// routes a packet to a neighbor based on the source route in the packet
	void sendPacket(KadPacket_p packet, simtime_t delay= 0) override;
	/// routes a packet to a neighbor based on the  in the packet
	void sendPacket(KadDataPacket_p packet);
	/// chooses the contact to forward a request to according to the overlay metric.
	const KadContact& chooseNextHop(const KadContactList&, const OverlayKey& target);
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
	/// sends a find node call to find this node
	void sendFindNode(const OverlayKey& key, bool exact=false);
	/// sends a find node call for rediscovery, possibly using "not via" failed link
	void sendRediscovery(KadContact* failedcontact, uint8_t maxFanOut);

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
        R2Kademlia() : routingTable(neighborTable), cmp(myOverlayKey), rediscoverQueue(cmp), urgentRediscoverQueue(cmp) {};
	/** @name policy functions called by routing table */
	//@{
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) override;
	/** called when a new contact is created in the routing table*/
	virtual void onNewContact(KadContact_p contact) override;
	/** called when a contact is going to be deleted from routing table*/
	virtual void onContactRemoval(KadContact_p contact) override;

	/** called when a route got improved */
	virtual void onImprovement(const KadContact& contact) override;
	/** called when physical neighbors may be out of synch with their sequence numbers **/
	virtual void reSyncPN(const KadContact& contact) override;
	/** called when supposed failed link detected **/
	virtual void onContactCheck(const KadContact& contact) override;
	/** called when new far away contact has been updated **/
	//virtual void askFingerForNeighbors(const KadContact& contact);

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
	pathseqno_t getGLSSeqNo() const { return glsseqno; }

	static constexpr unsigned long ContactAgeThreshold = 2000UL;

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
	std::unique_ptr<T> makeAnswer(const KadPacket& original);
	/// specialized version, calls also makeAnswer
	std::unique_ptr<KadPathSetupRsp> makeAnswerKadPathSetupRsp(const KadPathSetupReq& original);

	/// saves the contact list to the given packet.
	void fillRTableRsp(KadRTableRsp& packet, const KadContactList& contacts, bool includePaths=true);
	/// reads the contacts from the packet and adds the paths to the routing table
	void readRTableRsp(const KadRTableRsp& packet);
	//@}

	/** @name timer functions */
	//@{
	/// adds a timer to the timer vectors, sends a timer packet
	short addTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback);
	void stopTimer(short timerId);
	void stopAllTimers() { for (short timerId= 0; (size_type) timerId < timerMsg.size(); timerId++) { stopTimer(timerId); } };
	void doSendHello(short timerId);
	void doProbe(short timerId);
	void doProbeSelf(short timerId);
	void doProbeNeighbors(short timerId);
	void doProbeContacts(short timerId);
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
	bool checkForPathInstallation(KadPacket& packet);
	/// checks whether a PN requires a resynch and initiates PNDiscReq if so
	bool checkForResync(const OverlayKey& contact, stateseqno_t newseqno);
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

	// processes present Not Via information and returns true if own Node ID is already present
	bool processNotViaInformation(const KadPacket& packet);

	kad_path concatViaPath(const kad_path& viaPath, const OverlayKey& viaKey, const kad_path& path);
	/// adds contact and not via link for rediscovery queue (or send it immediately)
	void scheduleRediscovery(const OverlayKey& contactkey, const OverlayKey& notvia_unreachable_node, const OverlayKey& notvia_failednode_neighbor, bool immediate= false);
	/// schedule sending PNDiscReq
	void schedulePNDiscReq(const OverlayKey& neighbor);
	/// create and send PNDiscReqs for queued neighbors
	void sendScheduledPNDiscReq();

	void rediscoverContacts();
	/// adds routing table updates for contacts to an update queue
	void scheduleUpdate(const KadContact& affectedcontact, KadRouteUpdateAction action);
	/// send Update closer towards target contacts
	void propagateUpdateReq(std::unique_ptr<KadUpdateRouteReq> packet);
	/// sends all queued updates
	void sendUpdates();
	/// sends only KAD_Unreachable updates
	void sendUrgentUpdates();
	/// schedule a PathSetup
	void schedulePathSetup(const OverlayKey& contactkey);
	/// send all queue (valid) path setups
	void sendScheduledPathSetups();

	/// evaluate Updates while forwarding
	void evalUpdateReq(const KadUpdateRouteReq* packet);
	/// evaluateQueryRouteRsp
	void evalQueryRouteRsp(const KadQueryRouteRsp* packet) { readRTableRsp(*packet); }
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
	void incGLSSeqNo() { glsseqno++; }
private:
	~R2Kademlia() { updateQueue.clear(); pathSetupQueue.clear(); stopAllTimers(); }
	/// the overlay key of this node
	OverlayKey myOverlayKey;

	/// Node Global Link State Sequence Number
	pathseqno_t glsseqno= 0;
	/// Table of direct physical neighbors
	NeighborTable neighborTable;
	/// Kademlia-like Routing Table containing Paths to contacts
	KadRoutingTable routingTable;

	/// already queried neighbors
	std::unordered_set<OverlayKey> queriedNeighbors;
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
	uint8_t rediscoveryFanOutFactor= 2;
	simtime_t rediscoveryWaitTime;
	simtime_t pathSetupWaitTime;
	simtime_t updateWaitTime;
	simtime_t updateWaitTimeFast;
	bool detailedRoutingTableStats;
	//@}

	// a list that is used to send rediscover messages
	std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	// selfMessage
	bool updateScheduled= false;
	bool urgentUpdateScheduled= false;

	struct greatercmp {
		const OverlayKey& ownkey;
		greatercmp(const OverlayKey& key) : ownkey(key) {}
		bool operator()(std::unique_ptr<const KadContact>& leftcontact, std::unique_ptr<const KadContact>& rightcontact) const
			{ return (leftcontact.get()->getKey() ^ ownkey) > (rightcontact.get()->getKey() ^ ownkey);}
	} cmp;

	// a list that is used to send rediscover messages sorted by node ID vicinity
	typedef std::priority_queue<std::unique_ptr<const KadContact>, std::deque<std::unique_ptr<const KadContact>>, decltype(cmp)> rediscoverQueueType;
	rediscoverQueueType rediscoverQueue;
	rediscoverQueueType urgentRediscoverQueue;

	// a list that is used to send pathsetup messages
	std::list<OverlayKey> pathSetupQueue;

	// a list that is used to periodically probe neighbors
	std::list<OverlayKey> probeNeighborQueue;

	// a list that is used to periodically probe contacts
	std::list<OverlayKey> probeContactQueue;

	// a list that is used to send PN discover request messages
	std::list<OverlayKey> PNDiscoverQueue;

	// selfMessage
	bool pathSetupScheduled= false;

	// selfMessage
	bool rediscoveryScheduled= false;
	bool urgentRediscoveryScheduled= false;
	
	// selfMessage
	bool pndiscScheduled= false;
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

	/** @name periodic timer things */
	//@{
	std::vector<simtime_t> timerDuration;
	std::vector<cMessage *> timerMsg;
	//@}
};


inline
bool R2Kademlia::processNotViaInformation(const KadPacket& packet)
{
	bool ownIDFound(false);

	// first check the notvia information
	if (packet.getNotVia())
	{
		for (uint i= 0; i < packet.getFailedLinkArraySize(); i++) {
			auto &[unreachablenode, detectingnode]= packet.getFailedLink(i);
			ASSERT(not unreachablenode.isUnspecified() and not detectingnode.isUnspecified());
			EV_TRACE << "Packet " << packet.getName() << " contains not via <" << unreachablenode.shortStr() << ',' << detectingnode.shortStr() << '>' << (packet.getFailedLinkAuth(i) ? " AUTH" : "") << endl;
			if (unreachablenode == myOverlayKey or detectingnode == myOverlayKey) {
				// we should never trust notVia information about our own physical links
				ownIDFound= true;
			}
			else if ( packet.getFailedLinkAuth(i) ) {
				// we only trust notvia to be current from nodes that indicate physical neighbor failure
				// we should invalidate all contacts with that link in the path to them before answering
				routingTable.invalidateContacts(packet.getFailedLink(i));
			}
			else {
				routingTable.checkContacts(packet.getFailedLink(i));
			}
		} // end for
	}

	return ownIDFound;
}


// template functions
template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(const KadContact* contact, const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one go
	packet->setSource(myOverlayKey);
	packet->setHopLimit(hopLimit);
	packet->appendRoute(contact);
	packet->setTargetSeqNo(contact->getStateSeqNo());
	packet->setStateSeqNo(getGLSSeqNo());
	return packet;
}


template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(const KadPacket& original) {
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
	answer->setStateSeqNo(getGLSSeqNo());
	answer->setTargetSeqNo(original.getStateSeqNo());
	// now copy the notvia information to let the source update its obsolete routes, too
	if (original.getNotVia()) {
		answer->setNotVia(true);
		answer->setFailedLinkArraySize(original.getFailedLinkArraySize());
		for (uint i= 0; i < original.getFailedLinkArraySize(); i++) {
			answer->setFailedLink(i,original.getFailedLink(i));
		}
		answer->setFailedLinkAuthArraySize(original.getFailedLinkAuthArraySize());
		for (uint i= 0; i < original.getFailedLinkAuthArraySize(); i++) {
			answer->setFailedLinkAuth(i,original.getFailedLinkAuth(i));
		}

	}
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
