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

#ifndef __ROUTINGSIM_KIRA_H_
#define __ROUTINGSIM_KIRA_H_

#ifndef NDEBUG
#define _DEBUG_ONLY(X) X
#else
#define _DEBUG_ONLY(X)
#endif

// std includes
#include <optional>

// omnetpp includes
#include "simtime.h"

// local includes
#include "BaseTypes.h"
#include "Protocol.h"
#include "StatisticSample.h"
#include "KadPackets_m.h"
#include "NeighborTable.h"
#include "NodeVicinityGraph.h"
#include "KadRoutingTable.h"
#include "FwdTable.h"
#include "RPC.h"
#include "TimerMsg.h"

#include "RequestQueue.h"
#include "RateLimitedQueue.h"

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
using NotViaList_t = std::list< std::pair<link_t,age_t> >; // link with relative time (age)
using NotViaListAbsTime_t = std::list< std::pair<link_t,simtime_t> >; // link with absolute time (timestamp)

enum reasoncode : uint8_t {
	unspecified = 0,
	hopLimitExceeded = 1, // should never happen as KIRA is loop free
	nextHopUnknown = 2,   // not used
	travelledDisabledLink = 3, // in case a packet is wrongly sent along a link that is disabled/broken
	erroneousSourceRoute= 4,        // current hop in source route is not correct
	routeFailureDeadEnd= 5, // no ID-wise progress possible
	missingNeighbor = 7,    // forwarding entry contains non existing next hop
	missingPID = 8,         // cannot find a corresponding (incoming) pathID
	rerouteDataSegmentLimitExceeded = 9,    // this reroute would require more than two segment headers
	nextHopUnknownDataFwd = 10, // next hop not available during forwarding a data packet
	unhandledType= 128
};

enum linkFailureNotificationType : uint8_t {
	lfn_none = 0, // do not send linkFailureNotifications
	lfn_floodonly= 1, // only flood LFNs to the direct neighbors
	lfn_flood_and_directPID= 2, // flood LFNs to the direct neighbors and send to first segment sources directly via PID
	lfn_full= 3 // flood LFNs to the direct neighbors and notify all first segments source, even via overlay
};


enum probeStrategy_t : uint8_t {
  probeStrategy_none = 0,
  probeStrategy_randomID = 1,
  probeStrategy_existingID = 2,
  probeStrategy_bucketExpand = 3,
  probeStrategy_randomSuffix = 4,
  probeStrategy_suffixExpand = 5,
  probeStrategy_randomBucket = 6,
  probeStrategy_randomIDandBucketMixed = 7
};

// OverlayKey length 112 bit (16 bit prefix will be prepended to make it an IPv6 address)
constexpr uint8_t OverlayKeyLength= 112;

using ContactRequestQueue = RequestQueue<KadContact> ;
using KeyRequestQueue = RequestQueue< std::pair<OverlayKey,RequestQueueElementActionType> >;

/**
 * Implementation of KIRA – Kademlia-directed ID-based Routing Architecture
 * Initial publication:
 * R. Bless, M. Zitterbart, Z. Despotovic and A. Hecker, "KIRA: Distributed
Scalable ID-based Routing with Fast Forwarding"
 * 2022 IFIP Networking Conference (IFIP Networking), Catania, Italy, 2022,
 * doi: 10.23919/IFIPNetworking55013.2022.9829816.
 * https://ieeexplore.ieee.org/document/9829816
 *
 * Routing Tier: R²/Kad – Routing with R/Kademlia
 * Forwarding Tier: PathID-based Forwarding with Two Segments
 *
 * @author Jakob Thomas, Roland Bless
 */



class R2Kademlia : public RPCProtocol, public KadPolicy, public StatisticSample
{
private:

	~R2Kademlia();
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
	static constexpr int KAD_CTRL_PACKET_TYPE = 5;
	static constexpr int KAD_DATA_PACKET_TYPE = 6;

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
	bool useLinkLayerIndication;
	short probeTimerId;
	probeStrategy_t probeStrategy = probeStrategy_none;
	uint8_t rediscoveryFanOutFactor;
	uint8_t rediscoveryMaxNotification;
	uint8_t updateMaxNotification;
	uint8_t updateNotifyStrategy;
	uint8_t updatePropagateStrategy;
	uint8_t floodStrideFactor;
	bool floodCompleteBucketOpt;
	bool floodOnFailure;
	simtime_t PNRediscoveryWaitTime;
	simtime_t urgentRediscoveryWaitTime;
	simtime_t regularRediscoveryWaitTime;
	simtime_t slowRediscoveryWaitTime;
	simtime_t pathSetupHoldTime;
	simtime_t updateHoldTime;
	simtime_t updateHoldTimeFast;
	simtime_t minHelloInterval;
	simtime_t maxHelloInterval;
	bool detailedRoutingTableStats;
	bool detailedKIRAMessageStats;
	linkFailureNotificationType linkFailureNotifications;
	// send reachability test packets only to nodes with failed links
	bool failureFocussedProbing;
	bool addGratuitousContacts;
	// Number of tries to find a connected node as test packet destination
	unsigned int maxReachablityTries;
	//@}

	// to notify neighbors about affected PathIDs after a failure
	std::unordered_set<PathID>* notifyPIDs_p = nullptr;
	std::unordered_map<PathID, std::unordered_set<OverlayKey>> notifyFirstSegmentSources;
	std::unordered_map<OverlayKey, std::pair<OverlayKey, PathID>> notifyFirstSegmentSourcesReversePath; // target, <outgoingIF, PathID to target>
	std::unordered_map<OverlayKey, std::unordered_set<PathID>> incomingIFtoPIDmap;


	inline void addIncomingIFtoPIDEntry(const OverlayKey& incomingULN, PathID pidentry);
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
	rediscoverQueueClass ULNRediscoverQueue;
	rediscoverQueueClass urgentRediscoverQueue;
	rediscoverQueueClass regularRediscoverQueue;
	rediscoverQueueClass slowRediscoverQueue;

	// a list that is used to send pathsetup messages
	std::list<OverlayKey> pathSetupQueue;

	// a list that is used to periodically probe neighbors
	std::list<OverlayKey> probeNeighborQueue;

	// a list that is used to periodically probe contacts
	std::list<OverlayKey> probeContactQueue;

	// a queue that is used to send ULN/ON discover request messages

	// queues for rate limited requests
	ContactRequestQueue ULNDiscoverQueue; // for sending ULNDiscReq
	ContactRequestQueue GetULNQueue;      // for sending QueryRouteReq (Physical Neighbors)
	ContactRequestQueue GetONQueue;      // for sending QueryRouteReq (Overlay Neighbors)
	ContactRequestQueue ContactCandidateProbeQueue; // for probing contact candidates
        KeyRequestQueue OnDemandPathProbeQueue; // for sending path probes on demand (in contrast to periodical ones)

	RateLimitedQueue FloodQueue;
	RateLimitedQueue NotifyQueue;

	bool localLinkFailureNotificationsScheduled = false;
        // selfMessage
	bool pathSetupScheduled= false;

	// timestamp for outstanding ULNDiscReqs
	simtime_t LastPNDiscReq;
	// timestamp for outstanding PNQueryRouteReqs
	simtime_t LastPNQueryRouteReq;
	// timestamp for outstanding ONQueryRouteReqs
	simtime_t LastONQueryRouteReq;
	// timestamp for outstanding On Demand PathProbe
	simtime_t LastOnDemandProbeReq;
	// timestamp for outstanding contact Candidate PathProbe
	simtime_t LastContactCandidateProbeReq;

	// state seq number sent with last ULNHello
	stateseqno_t lastPNHelloStateSeqNo= stateseqno_t::invalidSeqNo;

	/** @name statistics */
	//@{
	uint forwardedPackets = 0;
	uint discardedPackets = 0;
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
	uint retriesRediscovery = 0;
	uint inconsistentPathHit = 0;
	uint sentRouteUpdates = 0;
	uint receivedRouteUpdates = 0;
	uint sentPathSetup = 0;
	uint sentPathTeardown = 0;
	uint receivedPathSetup = 0;
	uint forwardedPathSetup = 0;
	uint forwardedPathTeardown = 0;
        uint receivedPathTeardown = 0;

        struct msg_stats {
		unsigned long sentCount = 0UL;
		unsigned long receivedCount = 0UL;
		unsigned long forwardedCount = 0UL;
		unsigned long droppedCount = 0UL;
		unsigned long sentVolume = 0UL;
		unsigned long receivedVolume = 0UL;
		unsigned long forwardedVolume = 0UL;
        };
	// per message type statistics
        std::unordered_map<uint8_t, msg_stats> msgStats;

	uint floodPacketsReceived = 0;
	uint floodPacketReplications = 0;
	uint floodPacketMaxReplications = 0;
	//@}

	unsigned long laterpaths = 0;
	unsigned long shortenedlaterpaths = 0;
	unsigned long viablePIDpaths = 0;

	/** @name periodic timer things */
	//@{
	std::vector<simtime_t> randomTimerDuration;
	std::vector<simtime_t> fixedTimerDuration;
	std::vector<TimerMsg *> randomTimerMsg;
        std::vector<TimerMsg *> fixedTimerMsg;
	static constexpr short oneShotTimerId = std::numeric_limits<short int>::max();
	static constexpr short fixedTimerIdBase = std::numeric_limits<short int>::max()/2;
	short PNHelloTimerId= 0;
	//@}

	/** @name message generation */
	//@{
	/// crafts a hello packet and encapsulates it in a NodePkt
	NodePkt_p makeHelloPacket();
	/// crafts a packet of type T, sets the route to the given contact.
	template<typename T>
	std::unique_ptr<T> makePacket(const KadContact* contact, const char *name=nullptr, bool append_route= true);
	template<typename T>
	std::unique_ptr<T> makePacket(const KadContact& contact, const char *name=nullptr, bool append_route= true){
		return makePacket<T>(&contact, name, append_route);
	};
	/// crafts a data packet, sets the DestNodeID to first argument and also PathID according to the given viacontact.
	template<typename T>
	std::unique_ptr<T> makeDataPacket(const OverlayKey& dstNodeID, const KadContact* viacontact, const char *name);
	/// crafts a packet of type T, sets the route and nonce according to original
	template<typename T>
	std::unique_ptr<T> makeAnswer(const KadPacket& original, const char* packetName= nullptr);
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
	short addRandomizedTimer(simtime_t initialOffset, simtime_t duration, TimerMsg::cbfunc_t callback);
	short addFixedTimer(simtime_t initialOffset, simtime_t duration, TimerMsg::cbfunc_t callback);
	inline TimerMsg* addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback) override;
	/// stops timer and cancels timer message (and may delete it!)
	void stopTimer(short timerId,  bool delete_timermsg= true);
	/// stops timer and cancel timer message, starts new timer with new duration
	void resetTimer(short timerId, simtime_t duration);
	void stopAllTimers() { for (short timerId= 0; (size_type) timerId < randomTimerMsg.size(); timerId++) { stopTimer(timerId); }; for (short timerIdx= 0; (size_type) timerIdx < fixedTimerMsg.size(); timerIdx++) { stopTimer(fixedTimerIdBase+timerIdx); } };
	//void inspectQueue(ProtocolPacketQueue& ppq) const;
	/// timer callbacks
	bool doSendHello(short timerId);
	bool doProbeRandomly(short timerId);
	bool doProbeSelf(short timerId);
	bool doProbeNeighbors(short timerId);
	bool doProbeContacts(short timerId);
	bool doDataProbe(short timerId);
	bool doReachabilityTest(short timerId);
	bool doSendExplorePathPacket(short timerId);
        bool doPathRefresh(short timerId);
	bool doCheckULNs(short timerId);
	//@}

	/** @name private helpers */
	//@{
	/// does statistics on received packet
	void countReceived(KadPacket& packet);
	/// does statistics on dropped packet
	void countDropped(KadPacket& packet, const OverlayKey& target, uint8_t reason);
	/// count sent packets (use times=p >1 for broadcast to p neighbors)
        void countSent(KadPacket& packet, unsigned int times= 1);
	/// count forwarded
        void countFwd(KadPacket& packet);
	/// does statistics on received data packet
	void countReceived(KadDataPacket& packet);
	/// does statistics on received flood packet
	void countReceived(KadFloodPacket& packet, uint32_t replications);
	/// does statistics on dropped data packet
	void countDropped(KadDataPacket& packet, const OverlayKey& target, uint8_t reason);
	/// try a shortcut route and modify source route of the packet
	void applyShortcut(KadPacket& packet, const OverlayKey& destination);
	/// determine if pathID installation is necessary (e.g., when processing path setup request)
	bool checkForPathIDInstallation(const KadPacket& packet);
	/// remove path ID if possible
	void checkForPathTeardown(const KadPacket& packet);

	/// for a new neighborhoodcontact, install/remove any known paths within discoveryRadius
	void vicinityPathIDCompletion(const KadContact& neighborhoodContact, bool removePrecalcEntry= false);

	/// call when a node is disconnected
	void nodeDisconnect(nodeid_t nid);
	/// checks whether a ULN requires a resynch and initiates ULNDiscReq if so
	bool checkForResync(const OverlayKey& contact, stateseqno_t newseqno);
	/**
	 * sends an error packet to the sender of the given packet
	 * the original packet is encapsulated on the error packet
	 * @errorType the error type for the error packet
	 */
	void returnToSender(uint8_t errorType, KadPacket_p packet, const OverlayKey& failedNextHopNodeID= OverlayKey::UNSPECIFIED_KEY);
	void notifySender(uint8_t errorType, KadDataPacket_p packet, const OverlayKey& failedNextHopNodeID);
	/**
	 * Tries to cast packet to type T.
	 * If this succeeds, the method handlePacket(T) is called, the given pointer
	 * is invalidated and we return true.
	 * If the cast fails, nothing happens and we return false.
	 */
	template<typename T>
	bool handleIfFitting(KadPacket_p& packet);

	template<typename T>
	bool handleIfFitting(KadDataPacket_p& packet);

	// processes present Not Via information and returns true if own Node ID is already present
	bool processNotViaInformation(const KadPacket& packet);

	// processes packets while forwarding
	bool intermediateNodeProcessing(KadPacket& packet);

	// processes data packets while forwarding
    // currently this is only used for capturing statistics per packet type
	bool intermediateNodeProcessing(const KadDataPacket& packet);

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
	bool handleULNDiscResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleFindNodeResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleRPCTimeOut(KadPacket_p answer, RPCState& rpcState);
	//@}
	void incLSSeqNo() { lsseqno++; }

	void notifyAffectedPathUsers(const OverlayKey &key);
	void sendLinkFailureNotifications(NotViaListAbsTime_t* notvialist_p, std::unordered_set<PathID>* notifyPIDList_p, OverlayKey excludeNode);

	bool sendLocalLinkFailureNotifications();
	void scheduleLocalLinkFailureNotification();
	bool doNotifyFirstSegmentSources(std::unordered_set<PathID>* upstreamPIDs, NotViaListAbsTime_t* notvialist);

	/// adds contact and not via link for rediscovery queue (or send it immediately)
	void scheduleRediscovery(const OverlayKey& contactkey, rediscoverQueueClass& rediscoveryQueue, simtime_t rediscoveryWaitTime);
	bool rediscoverContacts(RediscoveryState::rediscoverQ_type qtype, simtime_t rediscoveryWaitTime);
  protected:
	/** @name Protocol interface */
	//@{
	/// called at initialization time of the Protocol
	void onInitialize() override;
	/// called when a new node is connected
	void onConnect(nodeid_t nid) override;
	/// called when a node is disconnected (on link layer indication)
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
	virtual void handlePacket(std::unique_ptr<KadULNDiscReq> packet);
	virtual void handlePacket(std::unique_ptr<KadQueryRouteReq> packet);
	virtual void handlePacket(std::unique_ptr<KadUpdateRouteReq> packet);
	virtual void handlePacket(std::unique_ptr<KadFindNodeReq> packet);
	virtual void handlePacket(std::unique_ptr<KadRevPathFlood> packet);
	virtual void handlePacket(std::unique_ptr<KadLinkFailureNotify> packet);
	virtual void handlePacket(KadAnswer_p packet);
	virtual void handleDataPacket(NodePkt_p packet);
	virtual void handlePacket(std::unique_ptr<KadFloodPacket> packet);
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
	bool tryForwardToNextOverlayHopWithPID(KadDataPacket_p& packet, NodePkt_p& nodePkt, const OverlayKey& nextOverlayHop);

    /// replicate the flood packet in respective id range, returns number of replications sent
    uint32_t replicateFloodPacket(std::unique_ptr<KadFloodPacket>& packet);
    bool sendFloodPacket(std::optional<hop_t> event = std::nullopt, NodePkt_ps payload = nullptr); 
    uint32_t replicateFloodPacket(KadFloodPacket& packet);
    std::unique_ptr<KadFloodPacket> createReplicationPacket(KadFloodPacket& packet, KadContact_p contact, uint8_t floodPrefixLength);
    std::unique_ptr<KadFloodPacket> createFloodPacket(KadContact_p contact, uint8_t floodPrefixLength, NodePkt_ps payload = nullptr);

	void handleFloodPacketDropped(const KadFloodPacket& packet);
	void handleFloodPacketForwarded(const KadFloodPacket& packet, hop_t takenHop);
	
	/// sends a received packet again after the source route was manipulated
	void resendPacket(KadPacket_p packet);
	/// try to find alternative next overlay hop
	void reroutePacket(const OverlayKey& finaldest, KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay= 0);
	/// routes a packet to a neighbor based on the source route in the packet
	void sendPacket(KadPacket_p packet, simtime_t delay= 0) override;
	/// routes a packet to a neighbor based on the source route in the packet
	void sendCtrlPacket(simtime_t time_added, KadPacket_p packet);
	/// routes a packet to a neighbor based on the Segment PIDs  in the packet
	void sendPacket(KadDataPacket_p packet);
	KadContactList::const_iterator chooseNextHop(const KadContactList& contacts, const OverlayKey& target, KadContactList::const_iterator startit);
	/// chooses the contact to forward a request to according to the overlay metric.
	const KadContact& chooseNextHop(const KadContactList&, const OverlayKey& target);
	/// sends a probe request to the given contact to check for path validity
	void sendPathProbeToContact(const KadContact& contact, RequestQueueElementActionType action);
	void sendPathProbe(const OverlayKey& contactkey);
	/// sends a path setup request to the given contact
	void sendPathSetup(KadContact_p contact);
	/// sends a path teardown request to the given contact
	void sendPathTeardown(KadContact_p contact);
	/// sends a "get physical neighbors" call to the contact
	simtime_t sendGetPhysicalNeighbors(const KadContact& contact, RequestQueueElementActionType action= unset);
	/// sends a "get overlay neighbors" call to the contact
	simtime_t sendGetOverlayNeighbors(const KadContact& contact, RequestQueueElementActionType action);
	/// sends a ULNDisc Request to the Physical Neighbor
	simtime_t sendDiscoveryReq(const KadContact& contact, RequestQueueElementActionType action);
	/// sends a Probe Request to the contact
	simtime_t sendOnDemandPathProbe(KeyRequestQueue::value_type keyactionpair, RequestQueueElementActionType unused_arg);
	/// sends a find node call to find a path this node
	void sendFindNode(const OverlayKey& key, bool exact=false, bool track=false);
	/// sends a find node call via viacontactkey to find a path to this node
	void sendFindNodeVia(const OverlayKey& destkey, KadContact_p viacontactkey, bool exact, bool track);
	/// sends a find node call for rediscovery, possibly using "not via" failed link, last two parameters required for scheduling retries
	bool sendRediscovery(OverlayKey failedcontact, uint8_t maxFanOut, simtime_t rediscoveryWaitTime);

	/// sends a data packet
	void sendData(const OverlayKey& key, NodePkt_ps payload = nullptr);
	//@}

	/// schedule rediscovery
	void scheduleULNRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, ULNRediscoverQueue, PNRediscoveryWaitTime); };
	void scheduleUrgentRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, urgentRediscoverQueue, urgentRediscoveryWaitTime); };
	void scheduleRegularRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, regularRediscoverQueue, regularRediscoveryWaitTime); };
	void scheduleSlowRediscovery(const OverlayKey& contactkey) override { scheduleRediscovery(contactkey, slowRediscoverQueue, slowRediscoveryWaitTime); };

	ContactRequestQueue::iterator scheduleRequest(ContactRequestQueue& q, KadContact&& tmpcontact);
	/// schedule sending ULNDiscReq
	void scheduleULNDiscReq(KadContact&& tmpcontact) { scheduleRequest(ULNDiscoverQueue, std::move(tmpcontact)); };
	void scheduleGetULNReq(KadContact&& tmpcontact) { scheduleRequest(GetULNQueue, std::move(tmpcontact)); };
	void scheduleGetONReq(const KadContact& tmpcontact);
	void scheduleContactCandidateProbe(KadContact&& tmpcontact) { scheduleRequest(ContactCandidateProbeQueue, std::move(tmpcontact)); }
        // schedule on demand ProbeReq
	void scheduleOnDemandPathProbeReq(KeyRequestQueue& q, const KadContact& tmpcontact, RequestQueueElementActionType action);
	/// schedule sending ULNDiscReq
	bool sendScheduledULNDiscReqs() { sendScheduledRequests(ULNDiscoverQueue); return false; };
	bool sendScheduledQueryRouteReqsULN() { sendScheduledRequests(GetULNQueue); return false; };
	bool sendScheduledQueryRouteReqsON() { sendScheduledRequests(GetONQueue); return false; };
	bool sendScheduledRequests(ContactRequestQueue& q);

	void sendScheduledContactCandidateProbeReqs(ContactRequestQueue& q);
	bool sendScheduledContactCandidateProbes() { sendScheduledContactCandidateProbeReqs(ContactCandidateProbeQueue); return false; }
	simtime_t sendPathProbeToContactCandidate(const KadContact& tmpcontact, RequestQueueElementActionType action);

	bool sendScheduledOnDemandPathProbe() { sendScheduledPathProbeRequests(OnDemandPathProbeQueue); return false; };
	void sendScheduledPathProbeRequests(KeyRequestQueue& q);

	/// adds routing table updates for contacts to an update queue
	void scheduleUpdate(const KadContact& affectedcontact, KadRouteUpdateAction action);
	/// forwards packet close to destcontactKey using the closest overlay neighbors
	void forwardCloser(KadPacket_p packet, const OverlayKey& destContactKey);
	/// for PID-based forwarding of LinkFailureNotify messages
	void forwardLinkFailureNotify(std::unique_ptr<KadLinkFailureNotify> linkfailurenotifypkt, NodePkt_p pkt);
	/// send Update closer towards target contacts
	void propagateUpdateReq(std::unique_ptr<KadUpdateRouteReq> packet);
	/// sends all queued updates
	bool sendUpdates();
	/// sends only KAD_Unreachable updates
	bool sendUrgentUpdates();
	/// schedule a PathSetup
	void schedulePathSetup(const OverlayKey& contactkey);
	/// send all queue (valid) path setups
	bool sendScheduledPathSetups(short timerId);
	/// perform garbage collection of unused PathIDs
	bool doGarbageCollectPathIDs(short timerId);

	/** @name policy foo */
	//@{
	void onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt, const OverlayKey& missingNeighborNodeID);
	void onMissingNeighbor(KadDataPacket_p packet, const OverlayKey& missingNeighborNodeID);
	/** Called when an error packet is encountered, updates routing information */
	bool onErrorPacket(KadError& error);
	//@}
	virtual void sampleStatistics (Statistics &stats) override;
public:
		R2Kademlia();
		virtual void finish() override { fwdTablePathID.garbageCollect(notifyFirstSegmentSources,notifyFirstSegmentSourcesReversePath, 3*R2Kademlia::PathRefreshThreshold); RPCProtocol::finish(); stopAllTimers(); }
	/** @name policy functions called by routing table */
	//@{
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) override;
	/** called when a new contact is created in the routing table*/
        virtual void onNewContact(KadContact_p contact) override;
	/** called for contact candidates **/
	virtual void onContactCandidate(std::unique_ptr<KadContact> contactp) override;
	/** called when a contact is going to be deleted from routing table*/
	virtual void onContactRemoval(KadContact_p contact) override;
	/** called when a contact is invalidated */
	virtual void onContactInvalidation(KadContact_p contact) override;
	/** called when path has changed, but is not improved **/
	virtual void onPathChange(const KadContact& contact) override;
	/** called when a route got improved (shorter or repaired) */
	virtual void onImprovement(const KadContact& contact) override;
	/** called when physical neighbors may be out of synch with their sequence numbers **/
	virtual void reSyncULN(const KadContact& contact, stateseqno_t expected_seqno) override;
	/** sendGetPhysicalNeighbors to contact **/
	virtual void recheckPhysicalNeighbors(const KadContact& contact) override;
	/** called when supposed failed link detected to initiate path probing **/
	virtual void onPathCheck(const KadContact& contact, RequestQueueElementActionType action) override;
	//virtual void onFailedLinkInVicinity(const link_t& failedLink, age_t failedLinkAge) override;
	//@}

	uint getHopLimitDefault() const { return hopLimit; };

    // Public API
    bool sendFloodPacket(NodePkt_p pkt); 
    void sendDataPacket(const OverlayKey& finaldestkey, NodePkt_p payload); 

	/** @name statistics functions used by statistics module */
	//@{
	const KadRoutingTable& getRoutingTable() const { return routingTable; }
	const OverlayKey& getOverlayKey() const { return myOverlayKey; }
	uint getForwardedPackets() const { return forwardedPackets; }
	uint getDiscardedPackets() const { return discardedPackets; }
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
	static constexpr unsigned long PathInfoAgeThreshold = 2'000UL; // ms
	// refresh within [0.5,1.5]*PathRefreshThreshold, garbage collection after 3*PathRefreshThreshold
	static constexpr unsigned long PathRefreshThreshold = 30'000UL; // ms
	static constexpr unsigned long ProbeSelfMinInterval = 250; // ms
	static constexpr unsigned long notifyLinkFailureWaitTime = 10UL; // ms
	static constexpr unsigned long RediscoveryNextTryWaitTime = 100UL; // ms
	static constexpr unsigned long PNDiscoverInitialWaitTime = 1'000UL; // µs (1ms)
	static constexpr unsigned long PNDiscoverWaitTime = 10'000UL; // µs (10ms ~ 100 Requests/s)
	static constexpr unsigned long GetPNInitialWaitTime = 10'000UL; // µs
	static constexpr unsigned long GetPNWaitTime = 20'000UL; // µs (50 Requests/s)
	static constexpr unsigned long GetONInitialWaitTime = 20'000UL; // µs
	static constexpr unsigned long GetONWaitTime = 40'000UL; // µs (25 Requests/s)
	static constexpr unsigned long ContactCandidateInitialWaitTime = 40'000UL; // µs (40ms)
	static constexpr unsigned long OnDemandPathProbeReqInitialWaitTime = 100'000UL; // µs (100ms)
	static constexpr unsigned long OnDemandPathProbeReqWaitTime = 100'000UL; // µs (100ms ~ 10 Requests/s)
	static constexpr unsigned long notifyLinkFailurePauseTime = 2'500UL; // µs (2.5ms ~ 400 Requests/s)
	static constexpr unsigned long revPathFloodPauseTime = 1'000UL; // µs (1ms ~ 1000 Requests/s)

	static NotViaList_t emptyNotViaList;
};


// default constructor
inline R2Kademlia::R2Kademlia()
    : routingTable(neighborTable), cmp(myOverlayKey), ULNRediscoverQueue(cmp),
      urgentRediscoverQueue(cmp), regularRediscoverQueue(cmp),
      slowRediscoverQueue(cmp),
      ULNDiscoverQueue(
          "ULNDiscover",
          std::bind(&R2Kademlia::sendDiscoveryReq, this, ph::_1, ph::_2),
          std::bind(&R2Kademlia::sendScheduledULNDiscReqs, this),
          SimTime(PNDiscoverInitialWaitTime, omnetpp::SIMTIME_US)),
      GetULNQueue("QueryRouteReq(ULN)",
                 std::bind(&R2Kademlia::sendGetPhysicalNeighbors, this, ph::_1,
                           ph::_2),
                 std::bind(&R2Kademlia::sendScheduledQueryRouteReqsULN, this),
                 SimTime(GetPNInitialWaitTime, omnetpp::SIMTIME_US)),
      GetONQueue(
          "QueryRouteReq(ON)",
          std::bind(&R2Kademlia::sendGetOverlayNeighbors, this, ph::_1, ph::_2),
          std::bind(&R2Kademlia::sendScheduledQueryRouteReqsON, this),
          SimTime(GetONInitialWaitTime,omnetpp::SIMTIME_US)),
      ContactCandidateProbeQueue(
          "ContactCandidateProbeQueue",
          std::bind(&R2Kademlia::sendPathProbeToContactCandidate, this, ph::_1,
                    ph::_2),
          std::bind(&R2Kademlia::sendScheduledContactCandidateProbes, this),
          SimTime(ContactCandidateInitialWaitTime,omnetpp::SIMTIME_US)),
      OnDemandPathProbeQueue(
          "OnDemandPathProbeReq",
          std::bind(&R2Kademlia::sendOnDemandPathProbe, this, ph::_1, ph::_2),
          std::bind(&R2Kademlia::sendScheduledOnDemandPathProbe, this),
          SimTime(OnDemandPathProbeReqInitialWaitTime,omnetpp::SIMTIME_US)),
      FloodQueue(
          "FloodQueue", revPathFloodPauseTime, revPathFloodPauseTime,
          std::bind(&R2Kademlia::addFixedTimer, this, ph::_1, ph::_2, ph::_3),
          std::bind(&R2Kademlia::sendCtrlPacket, this, ph::_1, ph::_2)),
      NotifyQueue(
          "NotifyQueue", notifyLinkFailurePauseTime, notifyLinkFailurePauseTime,
          std::bind(&R2Kademlia::addFixedTimer, this, ph::_1, ph::_2, ph::_3),
          std::bind(&R2Kademlia::sendCtrlPacket, this, ph::_1, ph::_2))
{
};


/// for functions that only get called once and not periodically
/// @return pointer to timer message, e.g., for later cancellation
inline TimerMsg *
R2Kademlia::addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback) {
	auto msg = new TimerMsg(oneShotTimerId, callback);
	// callback will be carried out after delay
	scheduleAt(simTime() + delay, msg);
	return msg;
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
			//EV_TRACE << "Packet " << packet.getName() << " contains not via <" << unreachablenode.shortStr() << ',' << detectingnode.shortStr() << "> Age: " << packet.getFailedLinkAge(i) << endl;
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



inline
void
R2Kademlia::addIncomingIFtoPIDEntry(const OverlayKey& incomingULN, PathID incomingPID)
{
	EV_TRACE << "addIncomingIFtoPIDEntry(" << incomingULN.shortStr() << ", " << incomingPID << ")" << endl;
	if (auto it= incomingIFtoPIDmap.find(incomingULN); it != incomingIFtoPIDmap.end()) {
		it->second.insert(incomingPID);
	}
	else {
		incomingIFtoPIDmap.insert({incomingULN, {incomingPID}});
	}
}


// template functions
template<typename T>
std::unique_ptr<T> R2Kademlia::makePacket(const KadContact* contact, const char *name, bool append_route) {
	auto packet = std::make_unique<T>();
        if (name)
		packet->setName(name);
        else
		packet->setName(PDUNames.at(packet->getType()));
	// TODO optimization: do this in one pass
	packet->setSource(myOverlayKey);
	packet->setHopLimit(hopLimit);
	if (append_route)
		packet->appendRoute(contact);
	packet->setStateSeqNo(getLSSeqNo());
	packet->setSrcNodeDegree(neighborTable.size());
	return packet;
}


// this creates a Data packet and fills in the segment PathIDs
template<typename T>
std::unique_ptr<T> R2Kademlia::makeDataPacket(const OverlayKey& dstNodeID, const KadContact* viacontact, const char *name) {
	// viacontact is the next overlay hop we need to reach
	auto packet = std::make_unique<T>();
	packet->setName(name);
	packet->setType(KAD_Data);
	packet->setPathSrcNodeID(myOverlayKey);
	packet->setSrcNodeID(myOverlayKey);
	packet->setDstNodeID(dstNodeID);
	packet->setHopLimit(hopLimit);

	// since we only forward to known contacts, we also have a PID for them (unless PathSetup is in progress)
	// this PID may only lead to the Node three hops before the contact
	if ( auto pidentry= fwdTablePathID.NIDPIDTable.find(viacontact->getKey());
	     pidentry != fwdTablePathID.NIDPIDTable.cend() )
	{ // pidentry.second is incomingPathID
		// this will be used to lookup the outgoing PID in forwardDataPacket()
		packet->setFirstPathID(pidentry->second);
		// in case that we have a longer path, a second PID for the last path segment needs to be specified
		if ( auto secondpidentry= fwdTablePathID.NIDPIDTableLastSeg.find(viacontact->getKey());
		     secondpidentry != fwdTablePathID.NIDPIDTableLastSeg.cend() ) {
			packet->setSecondPathID(secondpidentry->second);
		}
	}
	else
	{
		EV_ERROR << "makeDataPacket() - no PathID found for via destination " << viacontact->getKey().shortStr() << endl;
	}
	return packet;
}


template<typename T>
std::unique_ptr<T> R2Kademlia::makeAnswer(const KadPacket& original, const char* packetName) {
	auto answer = std::make_unique<T>();
	if (packetName)
		answer->setName(packetName);
	else
		answer->setName(PDUNames.at(answer->getType()));
	answer->setSimplifiedReturnPathFrom(original);
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

template<typename T> bool R2Kademlia::handleIfFitting(KadPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	KadPacket *raw_ptr = packet.get();
	//EV_TRACE << "handleIfFitting raw_ptr " << static_cast<void*>(raw_ptr) << endl;
	std::unique_ptr<T> tPacket(dynamic_cast<T*>(raw_ptr));
	// if cast failed we get nullptr -> skip the rest, return false
        if (not tPacket) {
		// note: packet should not be delete here as handleIfFitting() is used with fall through calls
		return false;
        }
	// only after new encapsulation has been successful, control is now handed over to tPacket
	std::ignore = packet.release();
	//EV_TRACE << "handleIfFitting tPacket " << static_cast<void*>(tPacket.get()) << endl;
	// delegate responsibility to handlePacket method
	handlePacket(std::move(tPacket));
	return true;
}

template<typename T> bool R2Kademlia::handleIfFitting(KadDataPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	KadDataPacket *raw_ptr = packet.get();
	//EV_TRACE << "handleIfFitting raw_ptr " << static_cast<void*>(raw_ptr) << endl;
	std::unique_ptr<T> tPacket(dynamic_cast<T*>(raw_ptr));
	// if cast failed we get nullptr -> skip the rest, return false
	if (not tPacket) {
		// note: packet should not be delete here as handleIfFitting() is used with fall through calls
		return false;
	}
	// only after new encapsulation has been successful, control is now handed over to tPacket
	std::ignore = packet.release();
	//EV_TRACE << "handleIfFitting tPacket " << static_cast<void*>(tPacket.get()) << endl;
	// delegate responsibility to handlePacket method
	handlePacket(std::move(tPacket));
	return true;
}

} //namespace




#endif // __ROUTINGSIM_R2Kademlia_H_
