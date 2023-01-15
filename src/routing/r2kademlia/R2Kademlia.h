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
#include <functional>

#include "foundation.h"
#include "Protocol.h"
#include "StatisticSample.h"
#include "KadPackets_m.h"
#include "NeighborTable.h"
#include "NodeVicinityGraph.h"
#include "KadRoutingTable.h"
#include "FwdTable.h"
#include "RPC.h"

namespace routingsim {

// for std::bind with callback functions
namespace ph = std::placeholders;

// packet types
using KadDataPacket_p = std::unique_ptr<KadDataPacket>;
using KadPacket_p = std::unique_ptr<KadPacket>;
using KadAnswer_p = std::unique_ptr<KadAnswer>;
using KadHello_p = std::unique_ptr<KadHello>;
using KadError_p = std::unique_ptr<KadError>;

using UpdateList_t = std::list< std::pair<KadContact, KadRouteUpdateAction> >;

enum reasoncode : uint8_t {
	unspecified = 0,
        hopLimitExceeded = 1, // should never happen as KIRA is loop free
	nextHopUnknown = 2,   // not used
	travelledDisabledLink = 3, // in case a packet is wrongly sent along a link that is disabled/broken
	erroneousSourceRoute= 4,        // current hop in source route is not correct
	routeFailureDeadEnd= 5, // no ID-wise progress possible
	missingNeighbor = 7,    // forwarding entry contains non existing next hop
	missingPID = 8,         // cannot find a corresponding (incoming) pathID
	unhandledType= 128
};

// OverlayKey length 112 bit (16 bit prefix will be prepended to make it an IPv6 address)
constexpr uint8_t OverlayKeyLength= 112;

/**
 * Implementation of R2Kad.
 * Routing with R/Kademlia
 * @author Jakob Thomas, Roland Bless
 */


class R2Kademlia : public RPCProtocol, public KadPolicy, public StatisticSample
{
private:

	~R2Kademlia() { updateQueue.clear(); pathSetupQueue.clear(); }
	/// the overlay key of this node
	OverlayKey myOverlayKey;

	/// Node-wide Link State Sequence Number (unique inside the node)
	stateseqno_t lsseqno= 0;
	/// Table of direct physical neighbors
	NeighborTable neighborTable;
	/// Kademlia-like Routing Table containing Paths to contacts
	KadRoutingTable routingTable;

	/// n-hop vicinity for PathID calculation
	nodeVicinityGraph vicinity;
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
	short probeTimerId;
	uint8_t probeStrategy;
	uint8_t rediscoveryFanOutFactor;
	uint8_t rediscoveryMaxNotification;
	uint8_t updateMaxNotification;
	uint8_t updateNotifyStrategy;
	uint8_t updatePropagateStrategy;
	simtime_t PNRediscoveryWaitTime;
	simtime_t urgentRediscoveryWaitTime;
	simtime_t regularRediscoveryWaitTime;
	simtime_t slowRediscoveryWaitTime;
	simtime_t pathSetupWaitTime;
	simtime_t updateWaitTime;
	simtime_t updateWaitTimeFast;
	bool detailedRoutingTableStats;
	// send reachability test packets only to nodes with failed links
	bool failureFocussedProbing;
	//@}

	// a list that is used to send rediscover messages
	// UpdateList_t is std::list< std::pair<KadContact, KadRouteUpdateAction> >
	std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	// selfMessage
	bool updateScheduled= false;
	bool urgentUpdateScheduled= false;

	struct greatercmp {
		const OverlayKey& ownkey;
		greatercmp(const OverlayKey& key) : ownkey(key) {}
		bool operator()(const OverlayKey& leftcontact, const OverlayKey& rightcontact) const
			{ return (leftcontact ^ ownkey) > (rightcontact ^ ownkey);}
	} cmp;

	// a list that is used to send rediscover messages sorted by node ID vicinity
	typedef std::priority_queue< OverlayKey, std::deque<OverlayKey>, decltype(cmp)> rediscoverQueueType;
	class rediscoverQueueClass : public rediscoverQueueType {
	public:
		rediscoverQueueClass(decltype(cmp) comparefct) : rediscoverQueueType(comparefct), scheduled(false) {}
		container_type::iterator begin() { return c.begin(); }
		container_type::iterator end() { return c.end(); }
		bool isScheduled() const { return scheduled; }
		void setScheduled() { scheduled= true; }
		void clearScheduled() { scheduled= true; }
	private:
		bool scheduled;
	};
	rediscoverQueueClass PNRediscoverQueue;
	rediscoverQueueClass urgentRediscoverQueue;
	rediscoverQueueClass regularRediscoverQueue;
	rediscoverQueueClass slowRediscoverQueue;

	// a list that is used to send pathsetup messages
	std::list<OverlayKey> pathSetupQueue;

	// a list that is used to periodically probe neighbors
	std::list<OverlayKey> probeNeighborQueue;

	// a list that is used to periodically probe contacts
	std::list<OverlayKey> probeContactQueue;

	// a list that is used to send PN discover request messages
	typedef std::list<KadContact> RequestQueueType;
	class RequestQueue : public RequestQueueType {
	public:
		RequestQueue(const std::string descriptiontxt, std::function<simtime_t(KadContact_p)> sendrequestcall, std::function<void()> sendscheduledcall, unsigned long initialScheduleDelay) : sendrequestcall(sendrequestcall), sendscheduledcall(sendscheduledcall), initialScheduleDelay(initialScheduleDelay), description(descriptiontxt), scheduled(false) {};
		const std::string& getDescription() { return description; }
		bool isScheduled() const { return scheduled; }
		void setScheduled() { scheduled= true; }
		void clearScheduled() { scheduled= false; }
		std::function<simtime_t(KadContact_p)> sendrequestcall;
		std::function<void()> sendscheduledcall;
		unsigned long initialScheduleDelay = 0; // in µs
	private:
		const std::string description;
		bool scheduled = false;
	};

	// queues for rate limiting requests
	RequestQueue PNDiscoverQueue; // for sending PNDiscReq
	RequestQueue GetPNQueue;      // for sending QueryRouteReq (Physical Neighbors)
	RequestQueue GetONQueue;      // for sending QueryRouteReq (Overlay Neighbors)

	// selfMessage
	bool pathSetupScheduled= false;

	// timestamp for outstanding PNDiscReqs
	simtime_t LastPNDiscReq;
	// timestamp for outstanding PNQueryRouteReqs
	simtime_t LastPNQueryRouteReq;
	// timestamp for outstanding ONQueryRouteReqs
	simtime_t LastONQueryRouteReq;

	// state seq number sent with last PNHello
	stateseqno_t lastPNHelloStateSeqNo= stateseqno_t::invalidSeqNo;

	/** @name statistics */
	//@{
	uint forwardedPackets = 0;
	uint droppedPackets = 0;
	uint sentPackets = 0;
	uint receivedPackets = 0;
	unsigned long sentVolume = 0; /// sent data by control messages in bytes
	unsigned long receivedVolume = 0; /// received data by control messages in bytes
	unsigned long forwardedVolume = 0; /// forwarded data by control messages in bytes
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
	uint sentPathTeardown = 0;
	uint receivedPathSetup = 0;
	uint forwardedPathSetup = 0;
	uint forwardedPathTeardown = 0;
	uint receivedPathTeardown = 0;
	//@}

	unsigned long laterpaths = 0;
	unsigned long shortenedlaterpaths = 0;
	unsigned long viablePIDpaths = 0;

	/** @name periodic timer things */
	//@{
	std::vector<simtime_t> randomTimerDuration;
	std::vector<simtime_t> fixedTimerDuration;
	std::vector<cMessage *> randomTimerMsg;
	std::vector<cMessage *> fixedTimerMsg;
	short PNHelloTimerId= 0;
	//@}

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
	short addRandomizedTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback);
	short addFixedTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback);
	inline void addOneShotTimer(simtime_t delay, std::function<void()> callback) override;
	/// stops timer and cancels timer message
	void stopTimer(short timerId);
	/// stops timer and cancel timer message, starts new timer with new duration
	void resetTimer(short timerId, simtime_t duration);
	void stopAllTimers() { for (short timerId= 0; (size_type) timerId < randomTimerMsg.size(); timerId++) { stopTimer(timerId); }; for (short timerIdx= 0; (size_type) timerIdx < fixedTimerMsg.size(); timerIdx++) { stopTimer(-2-timerIdx); } };

	/// timer callbacks
	void doSendHello(short timerId);
	void doProbe(short timerId);
	void doProbeSelf(short timerId);
	void doProbeNeighbors(short timerId);
	void doProbeContacts(short timerId);
	void doDataProbe(short timerId);
	void doReachabilityTest(short timerId);
	void doSendExplorePathPacket(short timerId);
	void doRefreshSetupPaths(short timerId);
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
	void applyShortcut(KadPacket& packet, const OverlayKey& destination);
	/// determine if pathID installation is necessary (e.g., when processing path setup request)
	bool checkForPathIDInstallation(const KadPacket& packet);
	/// remove path ID if possible
	void checkForPathTeardown(const KadPacket& packet);

	/// for a new neighborhoodcontact, installed any known paths within discoveryRadius
	void vicinityPathIDCompletion(const KadContact& neighborhoodContact);

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

	// processes packets while forwarding
	bool intermediateNodeProcessing(const KadPacket& packet);

	kad_path concatViaPath(const kad_path& viaPath, const OverlayKey& viaKey, const kad_path& path);

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
	void incLSSeqNo() { lsseqno++; }
	/// adds contact and not via link for rediscovery queue (or send it immediately)
	void scheduleRediscovery(const OverlayKey& contactkey, rediscoverQueueClass& rediscoveryQueue, simtime_t rediscoveryWaitTime);
	void rediscoverContacts(RediscoveryState::rediscoverQ_type qtype, simtime_t rediscoveryWaitTime);
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
	KadContactList::const_iterator chooseNextHop(const KadContactList& contacts, const OverlayKey& target, KadContactList::const_iterator startit);
	/// chooses the contact to forward a request to according to the overlay metric.
	const KadContact& chooseNextHop(const KadContactList&, const OverlayKey& target);
	/// sends a probe request to the given contact to check for path validity
	void sendPathProbe(const OverlayKey& destinationNodeID);
	/// sends a path setup request to the given contact
	void sendPathSetup(KadContact_p contact);
	/// sends a path teardown request to the given contact
	void sendPathTeardown(KadContact_p contact);
	/// sends a "get physical neighbors" call to the contact
	simtime_t sendGetPhysicalNeighbors(KadContact_p contact);
	/// sends a "get overlay neighbors" call to the contact
	simtime_t sendGetOverlayNeighbors(KadContact_p contact);
	/// sends a PNDisc Request to the Physical Neighbor
	simtime_t sendDiscoveryReq(KadContact_p contact);
	/// sends a find node call to find a path this node
	void sendFindNode(const OverlayKey& key, bool exact=false, bool track=false);
	/// sends a find node call via viacontactkey to find a path to this node
	void sendFindNodeVia(const OverlayKey& destkey, KadContact_p viacontactkey, bool exact, bool track);
	/// sends a find node call for rediscovery, possibly using "not via" failed link, last two parameters required for scheduling retries
	void sendRediscovery(OverlayKey failedcontact, uint8_t maxFanOut, simtime_t rediscoveryWaitTime);

	/// sends a data packet
	void sendData(const OverlayKey& key);
	//@}

	/// schedule rediscovery
	void schedulePNRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, PNRediscoverQueue, PNRediscoveryWaitTime); };
	void scheduleUrgentRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, urgentRediscoverQueue, urgentRediscoveryWaitTime); };
	void scheduleRegularRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, regularRediscoverQueue, regularRediscoveryWaitTime); };
	void scheduleSlowRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, slowRediscoverQueue, slowRediscoveryWaitTime); };

	void scheduleRequest(class RequestQueue& q, const KadContact& tmpcontact);
	/// schedule sending PNDiscReq
	void schedulePNDiscReq(const KadContact& tmpcontact) { scheduleRequest(PNDiscoverQueue, tmpcontact); };
	void scheduleGetPNReq(const KadContact& tmpcontact) { scheduleRequest(GetPNQueue, tmpcontact); };
	void scheduleGetONReq(const KadContact& tmpcontact) { scheduleRequest(GetONQueue, tmpcontact); };
	/// schedule sending PNDiscReq
	void sendScheduledPNDiscReqs() { sendScheduledRequests(PNDiscoverQueue); };
	void sendScheduledQueryRouteReqsPN() { sendScheduledRequests(GetPNQueue); };
	void sendScheduledQueryRouteReqsON() { sendScheduledRequests(GetONQueue); };

	void sendScheduledRequests(class RequestQueue& q);

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
	/// perform garbage collection of unused PathIDs
	void doGarbageCollectPathIDs(short timerId);

	/** @name policy foo */
	//@{
	void onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt);
	void onMissingNeighbor(KadDataPacket_p packet, NodePkt_p pkt);
	/** Called when an error packet is encountered, updates routing information */
	void onErrorPacket(KadError& error);
	//@}
	virtual void sampleStatistics (Statistics &stats) override;
public:
        R2Kademlia();
	virtual void finish() override { fwdTablePathID.garbageCollect(); RPCProtocol::finish(); stopAllTimers(); }
	/** @name policy functions called by routing table */
	//@{
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) override;
	/** called when a new contact is created in the routing table*/
	virtual void onNewContact(KadContact_p contact) override;
	/** called when a contact is going to be deleted from routing table*/
	virtual void onContactRemoval(KadContact_p contact) override;
	/** called when a contact is invalidated */
	virtual void onContactInvalidation(KadContact_p contact) override;
	/** called when path has changed, but is not improved **/
	virtual void onPathChange(const KadContact& contact) override;
	/** called when a route got improved (shorter or repaired) */
	virtual void onImprovement(const KadContact& contact) override;
	/** called when physical neighbors may be out of synch with their sequence numbers **/
	virtual void reSyncPN(const KadContact& contact, stateseqno_t expected_seqno) override;
	/** sendGetPhysicalNeighbors to contact **/
	virtual void recheckPhysicalNeighbors(const KadContact& contact) override;
	/** called when supposed failed link detected to initiate path probing **/
	virtual void onPathCheck(const KadContact& contact) override;

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
	unsigned long getSentVolume() const { return sentVolume; }
	unsigned long getReceivedVolume() const { return receivedVolume; }
	unsigned long getForwardedVolume() const { return forwardedVolume; }

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
	stateseqno_t getLSSeqNo() const { return lsseqno; }

	// do not probe if information is newer than PathInfoAgeThreshold
	static constexpr unsigned long PathInfoAgeThreshold = 2000UL; // ms
	static constexpr unsigned long PathRefreshThreshold = 120000UL; // ms
	static constexpr unsigned long RediscoveryNextTryWaitTime = 100UL; // ms
	static constexpr unsigned long PNDiscoverWaitTime = 1000UL; // µs
	static constexpr unsigned long PNHelloMinInterval = 200UL; // ms
	static constexpr unsigned long PNHelloMaxInterval = 30000UL; // ms
	static constexpr unsigned long GetPNWaitTime = 10000UL; // µs
	static constexpr unsigned long GetONWaitTime = 20000UL; // µs
};


// default constructor
inline
R2Kademlia::R2Kademlia() :
	routingTable(neighborTable), cmp(myOverlayKey),
	PNRediscoverQueue(cmp),
	urgentRediscoverQueue(cmp),
	regularRediscoverQueue(cmp),
	slowRediscoverQueue(cmp),
	PNDiscoverQueue("PNDiscover", std::bind(&R2Kademlia::sendDiscoveryReq, this, ph::_1), std::bind(&R2Kademlia::sendScheduledPNDiscReqs, this), PNDiscoverWaitTime),
	GetPNQueue("QueryRouteReq(PN)", std::bind(&R2Kademlia::sendGetPhysicalNeighbors, this, ph::_1), std::bind(&R2Kademlia::sendScheduledQueryRouteReqsPN, this), GetPNWaitTime),
	GetONQueue("QueryRouteReq(ON)", std::bind(&R2Kademlia::sendGetOverlayNeighbors, this, ph::_1), std::bind(&R2Kademlia::sendScheduledQueryRouteReqsON, this), GetONWaitTime)
{
};


// for functions that only get called once and not periodically
inline
void R2Kademlia::addOneShotTimer(simtime_t delay, std::function<void()> callback) {
	auto msg = new cMessage("TimerMsg", -1);
	msg->setContextPointer( reinterpret_cast<void *>( new auto(callback) ) );
	// callback will be carried out after delay
	scheduleAt(simTime() + delay, msg);
}


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
			EV_TRACE << "Packet " << packet.getName() << " contains not via <" << unreachablenode.shortStr() << ',' << detectingnode.shortStr() << "> Age: " << packet.getFailedLinkAge(i) << endl;
			if (unreachablenode == myOverlayKey or detectingnode == myOverlayKey) {
				// we should never trust notVia information about our own physical links
				ownIDFound= true;
			}
			else { // authoritative NotVia info has typically age=0ms
				routingTable.invalidateContacts(packet.getFailedLink(i), packet.getFailedLinkAge(i));
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
	packet->setStateSeqNo(getLSSeqNo());
	packet->setSrcNodeDegree(neighborTable.size());
	return packet;
}


template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(const KadPacket& original) {
	auto answer = std::make_unique<T>();
	answer->setName((std::string(original.getName()) + std::string("Answer")).c_str());
	answer->setShortened(answer->setSimplifiedReturnPathFrom(original));

	ASSERT(answer->getSource() == myOverlayKey);
	answer->setNonce(original.getNonce());
	answer->setStateSeqNo(getLSSeqNo());
	answer->setSrcNodeDegree(neighborTable.size());
	// now copy the notvia information to let the source update its obsolete routes, too
	if (original.getNotVia()) {
		answer->setNotVia(true);
		answer->setFailedLinkArraySize(original.getFailedLinkArraySize());
		for (uint i= 0; i < original.getFailedLinkArraySize(); i++) {
			answer->setFailedLink(i,original.getFailedLink(i));
		}
		answer->setFailedLinkAgeArraySize(original.getFailedLinkAgeArraySize());
		for (uint i= 0; i < original.getFailedLinkAgeArraySize(); i++) {
			answer->setFailedLinkAge(i,original.getFailedLinkAge(i));
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
