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

#include "ProtocolPacketQueue.h"
#include "Statistics.h"
#include "KIRA.h"
#include "KadObserver.h"
#include "FwdTable.h"
#include <utility>


namespace routingsim {

// callback function type
using cbfunc_t = std::function<void(short timerId)>;

Define_Module(R2Kademlia);

NotViaList_t R2Kademlia::emptyNotViaList={};

R2Kademlia::~R2Kademlia()
{
	delete notifyPIDs_p;
	notifyPIDs_p= nullptr;
	updateQueue.clear();
	pathSetupQueue.clear();

	// free static temporary buffers
	if (PathID::tmppathIDbuf != nullptr) {
		delete[] PathID::tmppathIDbuf; PathID::tmppathIDbuf= nullptr; PathID::tmppathIDbufSize=0;
	}

	// see KadPath.cc
	extern char *tmpHashSumBuf;
	extern unsigned long tmpHashSumBufSize;

	if (tmpHashSumBuf != nullptr) {
		delete[] tmpHashSumBuf; tmpHashSumBuf= nullptr; tmpHashSumBufSize= 0;
	}
}

	
void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	OverlayKey::random(myOverlayKey, getRNG(0));
	fwdTablePathID.setOwnNodeID(&myOverlayKey);
	vicinity.setRoot(myOverlayKey);
	getKadObserver().registerUp(myOverlayKey, node().getNodeId());

	// bind KIRA_CTRL_PACKET_TYPE
	// interval bounds for random delay per control message
	const uint64_t minCtrlPlaneProcessingDelay = par("minKIRAControlPlaneProcessingDelay").intValue(); // in ns
	const uint64_t maxCtrlPlaneProcessingDelay = par("maxKIRAControlPlaneProcessingDelay").intValue(); // in ns
	const simtime_t maxCPWaitTime = SimTime(par("KIRAControlPlaneProcessingQueueDelayLimit").intValue(), omnetpp::SIMTIME_MS);
	ASSERT(minCtrlPlaneProcessingDelay <= maxCtrlPlaneProcessingDelay);
	if (maxCtrlPlaneProcessingDelay > 0 and minCtrlPlaneProcessingDelay <= maxCtrlPlaneProcessingDelay) {
		bind(KAD_CTRL_PACKET_TYPE,
		     std::make_unique<ProtocolPacketQueue>( [this,minCtrlPlaneProcessingDelay,maxCtrlPlaneProcessingDelay](void){ return uniform(SimTime(minCtrlPlaneProcessingDelay,SIMTIME_NS),SimTime(maxCtrlPlaneProcessingDelay,SIMTIME_NS)); }, maxCPWaitTime.inUnit(SIMTIME_NS)/maxCtrlPlaneProcessingDelay ));
	} else {
		bind(KAD_CTRL_PACKET_TYPE);
	}

	// bind KIRA_DATA_PACKET_TYPE
	const uint64_t minDataPlaneProcessingDelay = par("minKIRADataPlaneProcessingDelay").intValue();
	const uint64_t maxDataPlaneProcessingDelay = par("maxKIRADataPlaneProcessingDelay").intValue();
	if (maxDataPlaneProcessingDelay > 0 and minDataPlaneProcessingDelay <= maxDataPlaneProcessingDelay ) {
		bind(KAD_DATA_PACKET_TYPE,
		     std::make_unique<ProtocolPacketQueue>( [this,minDataPlaneProcessingDelay,maxDataPlaneProcessingDelay](void){ return uniform(SimTime(minDataPlaneProcessingDelay,SIMTIME_NS), SimTime(maxDataPlaneProcessingDelay,SIMTIME_NS)); }, SimTime(10, SIMTIME_MS).inUnit(SIMTIME_NS)/maxDataPlaneProcessingDelay) );
	} else {
		bind(KAD_DATA_PACKET_TYPE);
	}

	ASSERT(par("accelerationFactor").intValue() > 0);
	ASSERT(par("bucketSize").intValue() > 1);
	ASSERT(par("discoveryRadius").intValue() > 0);
	discoveryRadius = (uint)par("discoveryRadius").intValue();

	routingTable.init(
		myOverlayKey, this,
		par("accelerationFactor").intValue(),
		par("bucketSize").intValue(),
		par("proximityNeighborSelection").boolValue(),
		par("enableSinglePathDiversity").boolValue(),
		discoveryRadius);

	ASSERT(par("findNodeNumber").intValue() > 0);
	findNodeNumber = (size_type) par("findNodeNumber").intValue();
	proximityRouting = par("proximityRouting").boolValue();
	hopLimit = (uint) par("hopLimit").intValue();
	pathSnooping = par("pathSnooping").boolValue();
	sendDataPackets = par("sendDataPackets").boolValue();
	usePathIDs = par("usePathIDs").boolValue();
	useLinkLayerIndication = par("useLinkLayerIndication").boolValue();
	if (sendDataPackets and not usePathIDs)
	{
		EV_WARN << "usePathIDs must be set if sendDataPackets is enabled, changed to usePathIDs=on" << endl;
		usePathIDs= true;
	}

	const auto probeStrategyStr = par("probeStrategy").stdstringValue();
	if (probeStrategyStr == "none")
		probeStrategy = probeStrategy_none;
	else if (probeStrategyStr == "randomID")
		probeStrategy = probeStrategy_randomID;
	else if (probeStrategyStr == "existingID")
		probeStrategy = probeStrategy_existingID;
	else if (probeStrategyStr == "bucketExpand")
		probeStrategy = probeStrategy_bucketExpand;
	else if (probeStrategyStr == "randomSuffix")
		probeStrategy = probeStrategy_randomSuffix;
	else if (probeStrategyStr == "suffixExpand")
		probeStrategy = probeStrategy_suffixExpand;
	else if (probeStrategyStr == "randomBucket")
		probeStrategy = probeStrategy_randomBucket;
	else if (probeStrategyStr == "randomIDandBucketMixed")
		probeStrategy = probeStrategy_randomIDandBucketMixed;
	else
		throw cRuntimeError("unknown probe strategy %s", probeStrategyStr.c_str());

	detailedRoutingTableStats = par("detailedRoutingTableStats").boolValue();
	detailedKIRAMessageStats = par("detailedKIRAMessageStats").boolValue();
	PNRediscoveryWaitTime = (simtime_t) (par("PNRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	urgentRediscoveryWaitTime = (simtime_t) (par("urgentRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	regularRediscoveryWaitTime = (simtime_t) (par("regularRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	slowRediscoveryWaitTime = (simtime_t) (par("slowRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	updateHoldTime = (simtime_t) (par("updateHoldTime").doubleValue() * 0.001);  // given in ms
	updateHoldTimeFast = (simtime_t) (par("updateHoldTimeFast").doubleValue() * 0.001);
	pathSetupHoldTime = (simtime_t) (par("pathSetupHoldTime").doubleValue() * 0.001);  // given in ms
	rediscoveryFanOutFactor = (uint) par("rediscoveryFanOutFactor").intValue();
	rediscoveryMaxNotification = (uint) par("rediscoveryMaxNotification").intValue();
	updateMaxNotification = (uint) par("updateMaxNotification").intValue();
	updateNotifyStrategy = (uint) par("updateNotifyStrategy").intValue();
	updatePropagateStrategy = (uint) par("updatePropagateStrategy").intValue();
	switch ((uint)par("linkFailureNotifications").intValue()) {
		case lfn_floodonly:
			linkFailureNotifications= lfn_floodonly;
			break;
		case lfn_flood_and_directPID:
			linkFailureNotifications= lfn_flood_and_directPID;
			break;
        case lfn_full:
			linkFailureNotifications= lfn_full;
			break;
		default:
			linkFailureNotifications= lfn_none;
	};
	failureFocussedProbing = par("failureFocussedProbing").boolValue();
	addGratuitousContacts = par("addGratuitousContacts").boolValue();
	floodStrideFactor = par("floodStrideFactor").intValue();
	floodCompleteBucketOpt = par("floodCompleteBucketOpt").boolValue();
	floodOnFailure = par("floodOnFailure").boolValue();

	maxReachablityTries = par("maxReachablityTries").intValue();

	// register this module for statistics
	getStatistics().addSample(this);
	minHelloInterval = SimTime(par("minHelloInterval").doubleValue(),SIMTIME_MS);
	maxHelloInterval = SimTime( par("maxHelloInterval").doubleValue(), SIMTIME_MS);
	simtime_t probeInterval = (simtime_t) par("probeInterval").doubleValue();
	simtime_t exploreInterval = (simtime_t) par("exploreInterval").doubleValue();
	simtime_t floodInterval = (simtime_t) par("floodInterval").doubleValue();

	// doProbeSelf is required for joining (will do exponential backoff)
	// calculation for first timer is: initialOffset - duration/2 + random([0..duration])
	// start after 100ms, use 250ms, 500ms, 1s, 2s, 4s, 8s, 16s, 32s, 64s, 128s, 256s, 300s as intervals
	probeTimerId= addRandomizedTimer(SimTime(225,SIMTIME_MS), SimTime(250,SIMTIME_MS), std::bind(&R2Kademlia::doProbeSelf, this, ph::_1));

	// regularly probe for closest (overlay/ID-wise)  neighbors
	addRandomizedTimer(std::max(probeInterval, SimTime(1000,SIMTIME_MS)), probeInterval*2, std::bind(&R2Kademlia::doProbeNeighbors, this, ph::_1));
	// regularly probe for non ULN contacts to test path liveliness and also refresh soft states in the forwarding tier
	addRandomizedTimer(std::max(probeInterval/2, SimTime(1000,SIMTIME_MS)), probeInterval, std::bind(&R2Kademlia::doProbeContacts, this, ph::_1));
	// regularly send FindNodeReq to _randomly_ chosen IDs, i.e., key-IDs but not node-IDs
	// the advantage is to discover new and better contacts and/or paths
	addRandomizedTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doProbeRandomly, this, ph::_1));

	// For path exploration
	if (exploreInterval > 0)
		addRandomizedTimer(exploreInterval*1.5, exploreInterval, std::bind(&R2Kademlia::doSendExplorePathPacket, this, ph::_1));
	// resend Hello packets to keep state sequence numbers in synch, note that is includes an exponential backoff (until maxHelloInteral, usually 30s max)
	PNHelloTimerId= addRandomizedTimer(minHelloInterval, minHelloInterval, std::bind(&R2Kademlia::doSendHello, this, ph::_1));

	// when we do not have link layer indications, we need to regularly check for dead ULNs
	if (not useLinkLayerIndication)
		addRandomizedTimer(3.5*maxHelloInterval,maxHelloInterval,std::bind(&R2Kademlia::doCheckULNs, this, ph::_1));

	if (usePathIDs) {
		// if PathRefreshThreshold is 20s, refresh is scheduled between [10s,30s]
		addRandomizedTimer(SimTime(PathRefreshThreshold/2,SIMTIME_MS), SimTime(PathRefreshThreshold,SIMTIME_MS), std::bind(&R2Kademlia::doPathRefresh, this, ph::_1));
		// garbageCollection would happen after 90s + every 10s
		addFixedTimer(SimTime(3*PathRefreshThreshold,SIMTIME_MS), SimTime(10000,SIMTIME_MS), std::bind(&R2Kademlia::doGarbageCollectPathIDs, this, ph::_1));
	}

	if (sendDataPackets)
	{
	  EV << "will send data packets after " << getSimulation()->getWarmupPeriod() << endl;
	  addRandomizedTimer(getSimulation()->getWarmupPeriod()+probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doDataProbe, this, ph::_1));
	}
    if (floodInterval > 0) {
	    addRandomizedTimer(floodInterval*1.5, floodInterval, [this](short i){return sendFloodPacket();});
    }

	// Reachability Test: check whether other _existing_ nodes can be reached (uses KadObserver)
	// this is additional artificial test traffic to evaluate the routing protocol and not necessary for its operation
	addRandomizedTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doReachabilityTest, this, ph::_1));

	// expose the following variables in GUI for inspection
	WATCH(myOverlayKey);
	WATCH(lsseqno);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);
	WATCH_OBJ(fwdTablePathID);

	// the following is not necessary, because onConnect will do this
	//sendToNeighbors(makeHelloPacket());
}


// link comes up (treat like a link layer indication "link established"), immediately send a ULNHello message
void R2Kademlia::onConnect( nodeid_t nid ) {

	if (not useLinkLayerIndication)
		return;

	// send a hello packet to new neighbor after some random delay (average minHelloInterval)
	addOneShotTimer(uniform(minHelloInterval/2,3*minHelloInterval/2), [this,nid](short int ignored)->bool {
		auto packet(makeHelloPacket());
		countSent(*(static_cast<KadHello*>(packet->getEncapsulatedPacket())));
		sendToNode(nid, std::move(packet));
		return true;
	} );
	// additionally, reset ULNHello timer if this fires too far away in the future
	if (randomTimerDuration[PNHelloTimerId] > 3*minHelloInterval)
		resetTimer(PNHelloTimerId, minHelloInterval);
}


// link is down (treat like link layer indication "link down")
void R2Kademlia::onDisconnect( nodeid_t nid ) {
	//EV_TRACE << "onDisconnect " << nid << endl;
	// if this node is taken down, the packets arrive after the reset
	if (neighborTable.empty() or (useLinkLayerIndication == false))
          return;

	nodeDisconnect(nid);
}


void R2Kademlia::nodeDisconnect(nodeid_t nid) {

	OverlayKey key = getKadObserver().getNodeKey(nid);
	ASSERT(not key.isUnspecified());
	if (neighborTable.getNodeId(key) != nid)
		return;
	//ASSERT(neighborTable.getNodeId(key) == nid );
	EV_TRACE << "disconnect of node " << key.shortStr() << " (" << nid << ")" << endl;
	// change in physical neighborhood increases the Sequence Number
	incLSSeqNo();
	neighborTable.removeNeighbor(key);
	// remove link from vicinity
	vicinity.removeLink(myOverlayKey,key);

	ASSERT(neighborTable.getNodeId(key) == undefined_node);
	// set contact invalid (was routingTable.removeContact(key))
	// since the state gets updated, contact lastSeen is updated in setContactInvalid()
	if (not routingTable.setContactInvalid(key)) {
		// this should only happen for nodes with very large degree
		EV_ERROR << "missing contact " << nid << " " << key << " on node " << node().getNodeId() << endl;
		return;
	}
	else {  // due to the if condition we know that routingTable.getContact(key) will not return nullptr
		auto contactp= routingTable.getContactMutable(key);
		// call policy function
		// this may try an alternative route as proposed path from the vicinity if applicable
		onContactInvalidation(contactp);
	}

	// contacts from routing table with this neighbor as next hop are not usable until rediscovered
	auto counter= routingTable.invalidateContactsPN(key);
	if (counter > 0)
		EV_TRACE << "invalidated " << counter << " contacts" << endl;

	// since an Underlay Neighbor has been changed, we notify our other neighbors if any left
	if (not neighborTable.empty() and randomTimerDuration[PNHelloTimerId] > minHelloInterval) {
		resetTimer(PNHelloTimerId, minHelloInterval);
	}

	// if this option is enabled, we flood a notification in reverse path directions
	if (linkFailureNotifications != lfn_none)
		notifyAffectedPathUsers(key);

	// Try to rediscover that contact
	// after grace period (otherwise more links may fail immediately after this)
	// but not if the only link has failed 
	//unsigned long threshold= 5UL * counter; // 5 corresponds to 20% (100/20) of the entries
	// it is promoted to urgent in case there are more than a certain percentage
	//if (threshold > routingTable.getNumContactsWithoutPNs())

	if (routingTable.getContact(key)->getNodeDegree() > 1) {

		// try to find an alternative route using rediscovery via the overlay
		scheduleULNRediscovery(key);
	}
	// send UpdateRouteReq w/ failed link to ID-wise closest Nodes
	// we can rely on routingTable.getContact(key) != nullptr, because of earlier setContactInvalid() call
	scheduleUpdate(*(routingTable.getContact(key)), KAD_Unreachable);

	if (floodOnFailure) {
        // send "LSA" after short delay
		addOneShotTimer(uniform(SimTime(1000,SIMTIME_US), SimTime(3000,SIMTIME_US)), [this,nid](short i) {
			return sendFloodPacket(std::make_optional(hop_t(nid,node().getNodeId())));
		});
	}
}


/// called on reception of a NodePkt for this protocol, i.e., an R2/Kad packet
/// START HERE: this is the main entry point into R2/Kad protocol logic
void R2Kademlia::onMessage( NodePkt_p pkt ) {

	ASSERT(pkt->getType() == KAD_CTRL_PACKET_TYPE || pkt->getType() == KAD_DATA_PACKET_TYPE);

	if ( pkt->getType() == KAD_DATA_PACKET_TYPE )
	{
		handleDataPacket(std::move(pkt));
		return;
	}

	// we should check whether the link via this message travelled is
	// still active, otherwise we should drop this message as it was affected
	// by the link failure, e.g., more precisely using (getDuration(), getArrivalTime())
	const hop_t taken_hop= pkt->getHop();
	//ASSERT(Node::getNode(this)->getNodeId() == taken_hop.to);

	ASSERT(pkt->hasEncapsulatedPacket());

	if (not getNetworkManager()->getTopology().isConnected(taken_hop.from,taken_hop.to))
	{
		KadPacket_p packet = std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));
		EV_WARN << "received packet (msg id: " << packet->getId() << ") via an inactive link, thus will discard" << endl;
		if (packet->getControlInfo() == nullptr)
			packet->setControlInfo(pkt.release());

		countDropped(*packet, packet->getTarget(), travelledDisabledLink);
		return;
	}

	KadPacket_p packet = \
		std::unique_ptr<KadPacket>(check_and_cast<KadPacket*>(pkt->decapsulate()));

	/*
	  EV_TRACE << "onMessage msgid:" << packet->getId() << " treeId:" << packet->getTreeId()
	      << " <" << packet->getName() << "> this="
		 << static_cast<void*>(packet.get()) << " c:" << packet->getCreationTime()
		 << " isTracked:" << packet->isTracked() << endl;
	*/


	// hello packets do not include my key
	if (handleIfFitting<KadHello>(packet))
		return;

	// special handling for PID-based forwarding of LinkFailureNotify messages
	if (packet->getType() == KAD_LinkFailureNotify) {

		if (const auto linkfailurenotifypkt(dynamic_cast<KadLinkFailureNotify*>(packet.get()));
		    linkfailurenotifypkt != nullptr and linkfailurenotifypkt->getDstPID() != PathID::UndefinedPathID) {
			packet->advanceProgress();
			std::unique_ptr<KadLinkFailureNotify> linkfailurenotifypkt_uptr(dynamic_cast<KadLinkFailureNotify*>(packet.release()));
			forwardLinkFailureNotify(std::move(linkfailurenotifypkt_uptr), std::move(pkt));
			return;
		}

	}

	// other packets should include the key in the source route

	// check for hop limit (last resort), normally KIRA is loop free
	if (packet->getHopLimit() == 0) {
		EV_ERROR << "Hoplimit reached @Node " << myOverlayKey.shortStr() << " source route=" << std::make_pair(packet->getSrcPath(),packet->getActiveVector()) << endl;
		// will simply drop this packet
		return;
	}
	else {
		packet->advanceProgress(); // increase hop count
	}

        if (packet->getCurrentHop() != myOverlayKey) {
		EV_ERROR << "dropping packet, current hop " << packet->getCurrentHop().shortStr()  << " in source route does not match!" << endl;
		//TODO how does this happen?
		countDropped(*packet, packet->getTarget(), erroneousSourceRoute);
		return;
	}
	// must process ULNDiscReq to set the neighbor table entry first
	if (handleIfFitting<KadULNDiscReq>(packet))
		return;

	// check for presence of carried "not via" information
	// and evaluate it (invalidate affected contacts)
	processNotViaInformation(*packet);

	if (handleIfFitting<KadRevPathFlood>(packet))
		return;

	if (packet->hasRemainingHops()) {
		// we are not the recipient (LinkFailureNotify needs special handling)

		// perform (optional) processing while forwarding to its ultimate destination
		// the processing of the packet may abort early here (PathSetupReq or PathTeardownReq)
		if (intermediateNodeProcessing(*packet) == true)
			return;
		// continue forwarding according to source route
		forwardPacket(std::move(packet), std::move(pkt));
		return;
	}
	else { // no hops left, packet is addressed to me (I might be an overlay hop though and not the final destination)

		packet->setControlInfo(pkt.release());

                // learn route from packet
                // (unless it is a reachability test packet that should not be used for improving information)
		//TODO optimization use isPathInteresting

		if (packet->getType() != KAD_ULNDiscRsp and not packet->isTracked()) {
			// for packets from physical neighbors: check need for resynchronization:
			// in case the packet indicates a newer state seq no, resynch with neighbor
			if (neighborTable.isNeighbor(packet->getSource()))
				checkForResync(packet->getSource(), packet->getStateSeqNo());

			// evaluate return path for routing table (packet came to us this way, so path was recently working)
			// Trusted Path Info (no need to check)
			// for ULNDiscRsp no need to add path here
			routingTable.addPath(packet->getSource(), packet->reversePath(packet->getIntermediatePath()), packet->getStateSeqNo(), 0, packet->getSrcNodeDegree(), true);
		}


		// these types may forward packets via overlay, so they count themselves
		if (handleIfFitting<KadFindNodeReq>(packet)) return;
		if (handleIfFitting<KadUpdateRouteReq>(packet)) return;
		if (handleIfFitting<KadLinkFailureNotify>(packet)) return;
		// the following types never forward packets
		countReceived(*packet);
		if (handleIfFitting<KadQueryRouteReq>(packet)) return;
		// handle RPC answer packets (including Errors)
		if (handleIfFitting<KadAnswer>(packet)) return;
		if (handleIfFitting<KadPathSetupReq>(packet)) return;
		if (handleIfFitting<KadPathTeardownReq>(packet)) return;
		if (handleIfFitting<KadProbeReq>(packet)) return;
		EV_ERROR << "dropping unhandled packet of type " << (int)packet->getType() << endl;
		countDropped(*packet, packet->getTarget(), unhandledType);
	}
}


void R2Kademlia::reset() {
	Enter_Method_Silent();
	EV << "resetting state of R2Kademlia" << endl;
	getKadObserver().registerDown(myOverlayKey, node().getNodeId());
	neighborTable.clear();
	routingTable.clear();
	unbind(KAD_CTRL_PACKET_TYPE);
	unbind(KAD_DATA_PACKET_TYPE);
	onInitialize();
	//TODO optimization: implement clear function in routing table
	//TODO clear timers
	//TODO clear rpc states
}



// process answer packets
void R2Kademlia::handlePacket(KadAnswer_p packet) {
	EV_DEBUG << "received " << packet->getName() << " (for nonce=" << packet->getNonce() << ") from " << packet->getSource().shortStr() << endl;
	if (packet->getType() == KAD_Error) {
		auto errorpkt_p(check_and_cast<KadError *>(packet.get()));
		if (onErrorPacket(*errorpkt_p)) {
			// if returned true we need to forward the Error packet closer to its final destination
			const auto finaldest( errorpkt_p->getDstNodeID() );
			forwardCloser(std::move(packet), finaldest);
			return;
		}
		// note that handleAnswer() will be called  afterwards for processing the RPC result
	}
	else if (packet->getType() == KAD_FindNodeRsp) {

		// update StateSeqNo and lastseen of origin contact
		if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
			// it might be the case that packet was reordered by using a different path than an earlier message
			if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
				contact_p->setStateSeqNo(packet->getStateSeqNo());
				contact_p->updateLastSeen();
				contact_p->setNodeDegree(packet->getSrcNodeDegree());
			}
			else
				EV_WARN << "State sequence number of message smaller (" << packet->getStateSeqNo() << ") than for contact (" << contact_p->getStateSeqNo() << ")" << endl;

		}

		if (packet->isTracked())
			getKadObserver().sampleAnswer(*packet);
		// try to shorten the return path (we exclude ourselves first)
		kad_path returnpath(packet->reversePath(packet->getSrcPath(0,packet->getProgress())));
		EV_TRACE << "answer packet: path to replier " << returnpath;
		laterpaths++;
		if (routingTable.shortenPath(returnpath, 0))
		{
			shortenedlaterpaths++;
			EV_TRACE << " shortened " << returnpath;
		}
		EV_TRACE << endl;
		if (packet->isTracked())
			getKadObserver().sampleLaterPath(getOverlayKey(),returnpath);

	}

	// the following call handles the RPC answer
	handleAnswer(std::move(packet)); // see RPC.cc
}


/// note: this method is also called when packets originate from this node
void R2Kademlia::forwardPacket(KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_ULNHello);
	ASSERT(packet->hasRemainingHops());
	ASSERT(packet->getControlInfo() == nullptr);

	bool packetorigin(packet->getProgress() == 0);
	// TODO Check for already present NotVia() => loop
	// TODO Check for presence of own ID in so far processed path (maybe as overlay hop)

	// optimization: potential shortcut - we check if the target is a known contact
	// only allowed for exact and not strict routing
	if (packet->getType() == KAD_FindNodeReq) {
		KadFindNodeReq* findnodepacket= dynamic_cast<KadFindNodeReq*>(packet.get());
		if (findnodepacket != nullptr and findnodepacket->getExact() == true) {
			applyShortcut(*findnodepacket,findnodepacket->getDstNodeID());
		}
	}

	// it is not allowed to use a reference for nextHop here,
	// because modRoute() may delete the referenced object and nextHop is used thereafter
	const OverlayKey nextHop(packet->getNextHop());
	nodeid_t nid(neighborTable.getNodeId(nextHop));

	if (packetorigin) { // only print this for originating packets
		EV_TRACE << "forwardPacket(): " << packet->getName() << " dest " << packet->getTarget().shortStr()
			 << " next hop: " << nextHop.shortStr() << " (" << nid << ')'
			 << " nonce=" << packet->getNonce()
			 << " spath " << packet->getSrcPath() << " progress=" << packet->getProgress() << " delay:" << delay << endl;
	}

	if (nid != undefined_node) {

		if (not (packetorigin or packet->isTracked())) {
			//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << " delay:" << delay << endl;
			countFwd(*packet);
		}

		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt), delay);

		return;
	}

	// next hop not available: for some packet types we can try to reroute
	bool try_to_reroute(false);
	switch (packet->getType()) {
	case KAD_UpdateRouteReq:
	case KAD_FindNodeReq:
	case KAD_LinkFailureNotify:
		try_to_reroute= true;
		break;
	// default:
	// try_to_reroute= false;
	// break;
	}; // end switch

	if (try_to_reroute) {
		// for Routing Updates and Rediscovery messages try to use overlay-based forwarding
		EV_TRACE << "forwardPacket(): next hop " << nextHop.shortStr() << " unavailable" << endl;

		const OverlayKey* finaldestination(nullptr);
		KadUpdateRouteReq* updatepacket(nullptr);
		KadFindNodeReq* findnodepacket(nullptr);
		KadLinkFailureNotify* linkfailurenotifypacket(nullptr);
		if (packet->getType() == KAD_UpdateRouteReq) {
			updatepacket= dynamic_cast<KadUpdateRouteReq*>(packet.get());
			if (updatepacket != nullptr)
				finaldestination= &updatepacket->getDstNodeID();
		}
		else
		if (packet->getType() == KAD_FindNodeReq) {
			findnodepacket= dynamic_cast<KadFindNodeReq*>(packet.get());
			if (findnodepacket != nullptr and findnodepacket->getRediscovery() == true) {
				finaldestination= &findnodepacket->getDstNodeID();
			}
		}
		else
		if (packet->getType() == KAD_LinkFailureNotify) {
			linkfailurenotifypacket= dynamic_cast<KadLinkFailureNotify*>(packet.get());
			if (linkfailurenotifypacket != nullptr) {
				finaldestination= &linkfailurenotifypacket->getDstNodeID();
			}
		}

		// Trying to forward to a next hop that is not found as physical next hop
		// or that was a  physical next hop, but is currently not available:
		// that will not fly, unless we have a valid replacement route for this next hop/contact
		// this is a rerouting just for the next hop
		if (KadContact_p contact(routingTable.getContact(nextHop));
		    contact != nullptr and contact->isValid()
		    and (updatepacket != nullptr or findnodepacket != nullptr)) // and contact.isPhysicalNeighbor()
		{
			// modify route
			kad_path newpath(contact->getPath());
			newpath.push_back(contact->getKey());
			bool detour= newpath.size() > 1;
			// put the rest of the path
			for (unsigned int i= packet->getProgress() + 2; i < packet->getRouteArraySize(); i++) {
				newpath.push_back(packet->getRoute(i));
			}
			// newpath may now contain a cycle, try to cut this
			// example: path <49d984 b20388 d65e04 2550d> is modified to
			//               < 49d984 b20388 f8d1e5 2550d5 d65e04 2550d5 4f5cc2 >
			// due to broken link < b20388 d65e04 > sustituted by <f8d1e5 2550d5 d65e04>
			// destination node is already contained in path
			routingTable.simplifyPath(newpath, {});
			EV_TRACE << "forwardPacket(): reroute ULN, contact " << *contact << endl;
			// rerouting makes this intermediate node to overlay hop
			packet->setActive(packet->getProgress(), true);
			packet->modRoute(newpath);

			EV_TRACE << "forwardPacket(): reroute path <";
			for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
				EV_TRACE << ' ' << packet->getRoute(i).shortStr();
			EV_TRACE << " > progress=" << packet->getProgress() << endl;

			if (detour) {
				// new next hop
				nid = neighborTable.getNodeId(newpath.front());
				if (nid != undefined_node) {
					// TODO check whether this actually makes sense for all packet types?
					packet->addNotViaLink({nextHop,myOverlayKey}, 0);
					EV_TRACE << "forwardPacket(): forwarding to next hop (" << nid << ")" << endl;

					if (not (packetorigin or packet->isTracked())) {
						//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << endl;
						countFwd(*packet);
					}

					nodePkt->encapsulate(packet.release());
					sendToNode(nid, std::move(nodePkt), delay);

					return;
				}
				else {
					EV_ERROR << "forwardPacket(): alternative next hop " << newpath.front().shortStr() << " also not valid" << endl;
				}
			}
			else
				EV_TRACE << "forwardPacket(): no alternative route for next hop found" << endl;
		}

		// only reroute UpdateRouteReq and FindNodeReq for rediscovery (TODO check)
		// try to find alternative route to final destination
		if (finaldestination) {
			EV_TRACE << "forwardPacket(): trying to reroute to final destination " << finaldestination->shortStr() << endl;
			// add not via information before forwarding
			if (  updatepacket != nullptr or findnodepacket != nullptr or linkfailurenotifypacket != nullptr )
			{
				packet->addNotViaLink({nextHop,myOverlayKey}, 0);
			}
			// need to delete the PID in case of rerouting via overlay
			if (linkfailurenotifypacket)
				linkfailurenotifypacket->setDstPID(PathID::UndefinedPathID);
			// try to reroute
			reroutePacket(*finaldestination, std::move(packet), std::move(nodePkt), delay);
			return;
		}
	} // end else try_to_reroute

	EV_ERROR << "forwardPacket(): could not find next hop!" << endl;
	onMissingNeighbor(std::move(packet), std::move(nodePkt), nextHop);
}


void R2Kademlia::reroutePacket(const OverlayKey& finaldest, KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay)
{
	NotViaList_t notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(std::make_pair(packet->getFailedLink(i), packet->getFailedLinkAge(i)));
		}
	}

	bool packetorigin(packet->getProgress()==0);

	// try up to k closest valid contacts
	KadContactList ovlyneighbors= routingTable.getClosestContacts(finaldest, routingTable.getSplitSize(), true, notvialist);
	EV_TRACE << "reroutePacket(): found " << ovlyneighbors.size() << " contacts close to " << finaldest << endl;

	while (not ovlyneighbors.empty())
	{
		const KadContact& contact= chooseNextHop(ovlyneighbors, finaldest);

		// if chooseNextHop() cannot find useful next hop: break
		if (contact.isNotValid() or contact.getKey().isUnspecified())
			break;

		if ((contact.getKey() ^ finaldest) < (myOverlayKey ^ finaldest))
		{ // we know a contact that is closer
			// it does not make sense to send a packet back to
			// an already passed overlay hop
			// (note that the rerouting node is one before the destination),
			// because we will get a loop and it's more likely that we have outdated information
			if ( packet->isOverlayHopInPath(contact.getKey()) or
			     (not contact.getPath().empty() and packet->isOverlayHopInPath(contact.getPath().back())) )
			{
				// delete contact from list
				if (auto it= std::find(ovlyneighbors.begin(), ovlyneighbors.end(), &contact); it != ovlyneighbors.end())
				{
					// since this pointer is a copy of the original bucket entry, we must not delete the object
					// set to nullptr to ignore it
					*it= nullptr;
				}
				else
				{
					EV_ERROR << "reroutePacket(): cannot find existing contact for deletion" << endl;
					break;
				}
				// next try
				continue;
			}

			// intermediate target must also be part of the path
			kad_path newpath(contact.getPath());
			newpath.push_back(contact.getKey());

			// rerouting makes this intermediate node to overlay hop
			packet->setActive(packet->getProgress(), true);
			packet->modRoute(newpath);
			EV_TRACE << "reroutePacket(): contact "
				 << contact <<	" resulting path <";
			for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
				EV_TRACE << ' ' << packet->getRoute(i).shortStr();
			EV_TRACE << " > progress=" << packet->getProgress() << endl;

			const OverlayKey& nextHop = packet->getNextHop();
			nodeid_t nid = neighborTable.getNodeId(nextHop);

			if (nid != undefined_node) {
				EV_TRACE << "reroutePacket(): " << packet->getName() << " dest " << packet->getTarget().shortStr()
					 << " next hop: " << nextHop.shortStr() << " (" << nid << ')' << endl;
				if (not (packetorigin or packet->isTracked())) {
					//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << endl;
					countFwd(*packet);
				}
				nodePkt->encapsulate(packet.release());
				sendToNode(nid, std::move(nodePkt), delay);

				return;
			}
			else
			{
				EV_TRACE << "reroutePacket(): cannot find alternative next hop" << endl;
				break;
			}
		}
		else
		{
			EV_TRACE << "reroutePacket(): cannot find alternative route via overlay (no progress)" << endl;
			break;
		}
	} // end while

	EV_ERROR << "reroutePacket(): no (more) contacts found" << endl;

	const OverlayKey nextHop(packet->getNextHop());
	// if we get here: bad luck
	onMissingNeighbor(std::move(packet), std::move(nodePkt), nextHop);
}


void R2Kademlia::forwardCloser(KadPacket_p packet, const OverlayKey& destContactKey)
{
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	NotViaList_t notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(make_pair(packet->getFailedLink(i),packet->getFailedLinkAge(i)));
		}
	}
	KadContactList closeContacts(routingTable.getClosestContacts(destContactKey, contactsLen, true, notvialist));

	if (closeContacts.empty()) {
		EV_TRACE << "forwardCloser(): forwarding ends here. No contact found" << endl;
		return;
	}

	const KadContact& contact = chooseNextHop(closeContacts, destContactKey);
	// check whether that contact is closer to the target than we are (but do not send back to source)
	if ( contact.isValid() and (contact.getKey() ^ myOverlayKey) > (contact.getKey() ^ destContactKey) and not packet->isOverlayHopInPath(contact.getKey()) )
	{
		packet->appendRoute(contact);
		EV_TRACE << "forwardCloser(): final dest: " << destContactKey.shortStr() << ", next dest: " << contact << endl;
	        resendPacket(std::move(packet));
	}
	else
	{
		EV_TRACE << "forwardCloser(): Update forwarding ends, final dest: " << destContactKey.shortStr() << ", next dest: " << contact << endl;
	}
	// if no chance of getting closer to the destination: packet will be deleted here!!
}


void R2Kademlia::resendPacket(KadPacket_p packet) {
	getKadObserver().registerPacketOverlayHop(*packet);
	ASSERT(packet->getControlInfo() != nullptr);
	NodePkt_p nodePkt = NodePkt_p(check_and_cast<NodePkt *>(packet->removeControlInfo()));
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
	nodePkt->setAttributes(nodePkt->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
	forwardPacket(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::sendPacket(KadPacket_p packet, simtime_t delay) {
	ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);

	if (not packet->hasRemainingHops() or packet->getTarget() == myOverlayKey) {
		EV_ERROR << "sendPacket: not sending packet destined to myself" << endl;
		return;
	}
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
        nodePkt->setType(KAD_CTRL_PACKET_TYPE);
        if (packet->isTracked()) // test packets for statistics will not be counted for link usage
		nodePkt->setAttributes(nodePkt->getAttributes() | NODEPKT_ATTR_DO_NOT_COUNT_ON_LINK);
	nodePkt->setAttributes(nodePkt->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
	if (packet->getType() == KAD_FindNodeReq) {
		const KadFindNodeReq* tmp_packet= check_and_cast<KadFindNodeReq*>(packet.get());
		// it doesn't make sense to count packets destined to isolated nodes: they cannot be reached
		if (tmp_packet->getExact() and tmp_packet->getRediscovery()==false and tmp_packet->isTracked()
		    and
		    getKadObserver().getPhysicalDistance(myOverlayKey, tmp_packet->getDstNodeID()) > 0) {
				getKadObserver().registerPacket(*packet);
				getStatistics().registerMessage(nodePkt.get());
		}
	}
	// EV_TRACE << "sendPacket(): " << packet->getName() << ", " << (int) packet->getType() << endl;

        // reachability test packets should not be counted
	if (not packet->isTracked()) {
		countSent(*packet);
	}
	forwardPacket(std::move(packet), std::move(nodePkt), delay);
}


// this sends a R²/Kad control message and corrects the notvia age by
// considering time passed since point in time time_added
void
R2Kademlia::sendCtrlPacket(simtime_t time_added, KadPacket_p packet)
{
  // need to adjust notvia timestamp by waiting time before sending
  if (packet->getNotVia()) {
	for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
          auto age_in_ms = packet->getFailedLinkAge(i);
          simtime_t timediff(simTime() > time_added + SimTime(1, SIMTIME_MS)
                                 ? simTime() - time_added
                                 : SIMTIME_ZERO);
	  EV_TRACE << "sendCtrlPacket: notvia adjustment Time added = " << time_added << ", now = " << simTime() << " timediff=" << timediff << "ms, as int:" << (unsigned long) timediff.trunc(SIMTIME_MS).inUnit(SIMTIME_MS) << "ms" << endl;
	  packet->setFailedLinkAge(i, age_in_ms+(unsigned long) timediff.trunc(SIMTIME_MS).inUnit(SIMTIME_MS));
	} // end for
  }
  sendPacket(std::move(packet));
}


/**
 * chooseNextHop performs proximity routing:
 * for contacts with the same XOR distance to target it selects the one with the shortest path
 * Assumption is that the contact list is sorted by XOR distance to target
**/
KadContactList::const_iterator
R2Kademlia::chooseNextHop(const KadContactList& contacts, const OverlayKey& target, KadContactList::const_iterator startit) {
	// if target falls into deepest bucket, use XOR metric instead of proximity routing
	// since the contact list is sorted by XOR, the ID-wise closest contact should be the first one
	if (routingTable.isInLastBucket(target)) {
		return startit;
	}

	if (proximityRouting and contacts.size() > 1) {
		// dprefix is the number of non-matching bits with target: smaller dprefix is closer to target
		int dprefix = target.getLength()+1; // keylength + 1
		size_t dproxMin = hopLimit;

		// look for first usable contact
		// since the contact list is sorted by XOR, the ID-wise closest contact should be the first one
		const KadContact* bestcontact(nullptr);
		auto it(startit);
		auto bestchoice_it(it);
		for (; it != contacts.cend(); it++)
		{
			bestcontact= *it;
			bestchoice_it= it;
			if (bestcontact != nullptr)
			{
				// initialize the prefix distance and proximity distance
				dprefix = (bestcontact->getKey() ^ target).log_2();
				dproxMin = bestcontact->getPathLength();
				//EV_TRACE << "chooseNextHop(): contact " << contact->getKey().shortStr()
				//	 << " dprefix=" << dprefix << " dproxMin=" << dproxMin << endl;

				// stop here as we have found a usable contact to start with
				break;
			}
		}

		// dprefix, dproxMin now initialized to first non-null contact (if any)
		// continue with next contact
		if (it != contacts.cend())
			it++;

		// look for closest contact with shortest path
	        for (; it != contacts.cend(); it++) {

			if ( (*it) != nullptr ) {
				//EV_TRACE << "chooseNextHop(): check contact " << (*it)->getKey().shortStr()
				//	 << " dprefix=" << ((*it)->getKey() ^ target).log_2() << " pathlen=" << (*it)->getPathLength() << endl;

				// stop at contact that is prefix-wise further away from target
				if (((*it)->getKey() ^ target).log_2() > dprefix)
					break;

				// contact is prefix-wise at the same distance (dprefix)
				// if contact has shorter path it is our best choice now
				if ((*it)->getPathLength() < dproxMin) {
					dproxMin = (*it)->getPathLength();
					bestcontact = *it;
					bestchoice_it = it;
				}
			}
		} // end for

		if (bestcontact != nullptr)
			return bestchoice_it;
	}
	else {
		// no proximity routing or only single contact

		if (not contacts.empty() and startit != contacts.cend() and *startit != nullptr) {
			if ((*startit)->getKey() == target)
			{
				EV_WARN << "Next hop is the target, number of contacts: " << contacts.size()
					<< "contact: " << *(startit) << endl;
			}
			return startit;
		}
	}

	// if everything else fails we return the end of the contact list
	// which signals that no eligible contact was found
	return contacts.cend();
}


/**
 * chooseNextHop performs proximity routing
 * Assumption is that the contact list is sorted by XOR distance to target
**/
const KadContact&
R2Kademlia::chooseNextHop(const KadContactList& contacts, const OverlayKey& target) {

	if (auto cit(chooseNextHop(contacts, target, contacts.cbegin())); cit != contacts.cend())
	{
		if (*cit != nullptr) {
			// dereference const_iterator, then dereference pointer
			return **cit;
		}
	}

	// if everything else fails we return an empty contact
	return KadContact::undefinedContact;
}

// -------------------- RPC methods ---------------------------


// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& destkey, bool exact, bool track) {
	EV_TRACE << "sendFindNode for " << destkey.shortStr() << (exact ? " (NID)" : (destkey == myOverlayKey ? " (JOIN)" : " (RID)")) << endl; // Node ID or Resource ID?

	//ASSERT(contacts.size() > 0);
	if (auto contacts = routingTable.getClosestContacts(destkey, findNodeNumber, true); contacts.size() > 0)
	{
		// use several contacts to send multiple Rediscovery requests in parallel
		const KadContact& contact = chooseNextHop(contacts, destkey);
		ASSERT(contact.isValid());
		ASSERT(!contact.getKey().isUnspecified());
		if (contact.getKey() == myOverlayKey)
		{
			EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
			return;
		}
		auto packet = makePacket<KadFindNodeReq>(contact);
		packet->setDstNodeID(destkey);
		packet->setExact(exact);
		if (track)
			packet->markTracked();
		EV_TRACE << "sending KadFindNodeReq" << endl;
		sendRpcCall(
			std::move(packet),
			std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
			std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
			std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
				);
	}
	else
	{ // no contacts to choose from (usually only possible for isolated nodes)
		EV_WARN << "no contacts found, cannot send FindNodeReq" << endl;
	}
}


// FIND NODE
void R2Kademlia::sendFindNodeVia(const OverlayKey& destkey, KadContact_p viacontact_p, bool exact, bool track) {
	EV_TRACE << "sendFindNode for " << destkey.shortStr() << (exact ? " (NID)" : (destkey == myOverlayKey ? " (JOIN)" : " (RID)"))
		 << " via contact " << viacontact_p->getKey().shortStr() << endl; // Node ID or Resource ID?

	if (viacontact_p == nullptr) {
		EV_ERROR << "viacontact null - internal error" << endl;
		return;
	}

	ASSERT(viacontact_p->isValid());
	ASSERT(!viacontact_p->getKey().isUnspecified());
	if (viacontact_p->getKey() == myOverlayKey)
	{
		EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
		return;
	}
	auto packet = makePacket<KadFindNodeReq>(*viacontact_p);
	packet->setDstNodeID(destkey);
	packet->setExact(exact);
	if (track)
		packet->markTracked();

	EV_TRACE << "sending KadFindNodeReq" << endl;
	sendRpcCall(
		std::move(packet),
		std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
		);

}


// handle incoming FindNodeReq
void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	const OverlayKey& query = packet->getDstNodeID(); // get the destination ID
	bool is_rediscovery= packet->getRediscovery();

	EV_TRACE << "KadFindNodeReq (id=" << packet->getId() << ", nonce=" << packet->getNonce() << ") received query for: " << query.shortStr()
		 << " isTracked=" << packet->isTracked() << (packet->getSource() == query ? " (JOIN)" : "") << (is_rediscovery ? " Rediscovery" : "") << endl;

	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	// construct notvia list for filtering contacts
	NotViaList_t notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(std::make_pair(packet->getFailedLink(i), packet->getFailedLinkAge(i)));
		}
	}
	// ContactList is sorted according to XOR metric with respect to key
	// the first contact is ID-wise closest to query key, however, it may not be the one with the shortest path
	KadContactList contacts = routingTable.getClosestContacts(query, contactsLen, true, notvialist);

	// remove source from the list of potential next hops to avoid loop or exclude it from answer
	if (!contacts.empty()) {
		// EV_TRACE << "Found contacts: ";
		// for (const auto& thiscontact : contacts)
		// {
		//  	EV_TRACE << thiscontact->getKey().shortStr() << " ";
		// }
		// EV_TRACE << endl;

		for (auto it= contacts.begin(); it != contacts.end(); it++) {
			if (*it != nullptr and (*it)->getKey() == packet->getSource()) {
				contacts.erase(it);
				// EV_TRACE << "removed " << packet->getSource() << " from found contacts" << endl;
				break;
			}
		}
	}

	if (query != myOverlayKey) { // someone else routing via us

		if (contacts.empty() or contacts.front()->getKey().isUnspecified())
		{
			EV_ERROR << "no contacts left, dead end." << endl;
			// since we need an exact match and cannot make any progress, we are lost here
			countDropped(*packet, query, routeFailureDeadEnd);
			returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

			if (is_rediscovery)
				droppedRediscovery++;

			return;
		}

		// EV_TRACE << "Found contacts: ";
		// for (const auto& thiscontact : contacts)
		// {
		//  	EV_TRACE << thiscontact->getKey().shortStr() << " ";
		// }
		// EV_TRACE << endl;

		// precondition here: !contacts.empty(), i.e., we have a contact as potential next hop
		// if next contact is closer to destination than us, it's ok unless we route back to the source
		if ((contacts.front()->getKey() ^ query) < (myOverlayKey ^ query)
		    and contacts.front()->getPathLength() < packet->getHopLimit()) {

			// we have found a contact that is closer than us to target (query)
			// => forward request
			// note: chooseNextHop may choose the next hop with the shortest path (proximity routing)
			const KadContact& contact= chooseNextHop(contacts, query);
			EV_TRACE << "KadFindNodeReq received query for: " << packet->getDstNodeID().shortStr() << " next overlay hop is " << contact << endl;
                        // note: the log_2 will be smaller for a longer common prefix
#ifndef NDEBUG
			ASSERT(&contact != &KadContact::undefinedContact);
                        ASSERT((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2());
			if((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
				EV_WARN << "no prefix progress made" << endl;
			}
#endif
                        // in rare cases, chooseNextHop may choose a ULNS contact
                        // that is further away than this node (e.g., many
                        // invalid contacts in higher buckets and the query is in such a higher bucket)
                        // then we would use the first contact instead as this makes progress
			if ((contact.getKey() ^ query) < (myOverlayKey ^ query))
				packet->appendRoute(contact);
			else
				packet->appendRoute(contacts.front());

                        ASSERT(packet->getRouteArraySize() - packet->getProgress() <= packet->getHopLimit());
			resendPacket(std::move(packet));
			if (is_rediscovery)
				forwardedRediscovery++;
			return;
		}
		else
		{
			// the best contact we know is further away from the target than we are
			// we must find the node exactly, however, if there is no progress from here
			// we should send back an error message
			if (packet->getExact()) {

				const KadContact& contact = chooseNextHop(contacts, query);
				EV_WARN << "best neighbor now: " << contact << endl;
				EV_TRACE << "distance for contact=" << (contact.getKey() ^ query).log_2()
					 << " own distance=" << (myOverlayKey ^ query).log_2() << " own ID: " << myOverlayKey.shortStr() << endl;
				if ((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
					EV_WARN << "no prefix progress made - dropping request" << endl;
				}

				if (contacts.front()->getPathLength() < packet->getHopLimit())
				{
					// since we need an exact match and cannot make any progress, we are lost here
					EV_ERROR << "cannot reach final destination " << query.shortStr() << " (dead end)" << endl;
					countDropped(*packet, query, routeFailureDeadEnd);

					// since this node should exist, it may be a hint for inconsistency
					// trigger discovery of own neighborhood (join) in case the contact is in our lowest bucket
					// and backoff has already advanced beyond 1s
					if (routingTable.getContact(query) != nullptr // contact may be present but is probably invalid already
					    and routingTable.isInLastBucket(query)
					    and randomTimerDuration[probeTimerId] > SimTime(1000,SIMTIME_MS)) {
						// send a join now
						sendFindNode(myOverlayKey, false);
						// reset timer for periodic joins to ask more frequently now
						resetTimer(probeTimerId, SimTime(ProbeSelfMinInterval,SIMTIME_MS));
					}

					returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));
				}
				else { // if we find a closer contact too far away, we should send HOP LIMIT EXCEEDED
					EV_ERROR << "closer contact " << contact.getKey() << " is too far away ("
						 << contacts.front()->getPathLength() << " hops)"
						 << endl;
					countDropped(*packet, query, hopLimitExceeded);

					returnToSender(KAD_HopLimitExceeded, std::move(packet));
				}

				if (is_rediscovery)
					droppedRediscovery++;
				return;

			} // if exact flag is set

			// otherwise it's ok to report what we know
			EV_TRACE << "Cannot get closer to the target, best neighbor now: " << contacts.front()->getKey() << endl;
			EV_TRACE << "Distance for contact=" << (contacts.front()->getKey() ^ query).log_2()
				 << " own distance=" << (myOverlayKey ^ query).log_2() << " own ID: " << myOverlayKey.shortStr() << endl;
		} // end else
	} // end if query!= myOverlayKey

	// if we get here:
	// either we are the query destination and thus should answer
	// or we cannot get closer to the destination and report what we have

	// answer request
	auto answer = makeAnswer<KadFindNodeRsp>(*packet);
	if (packet->isTracked()) {
		EV_TRACE << "packet was tracked, setTracked for answer" << endl;
		answer->markTracked();
        }
	countReceived(*packet);

	// add valid randomly chosen contacts from all other buckets to improve connectivity
        if (addGratuitousContacts)
	  routingTable.addRandomGratuitousContacts(contacts);

        fillRTableRsp(*answer, contacts);
	if (is_rediscovery) // rediscovery request successfully arrived at the destination
		succeededRediscovery++;

	EV_TRACE << "KadFindNodeReq query for " << query.shortStr() << " answering with list size " << contacts.size() << " Tracked: " << (answer->isTracked() ? 'T' : 'F') << endl;
	sendPacket(std::move(answer));
}


bool R2Kademlia::handleFindNodeResponse(KadPacket_p answer_, RPCState& rpcState) {
	ASSERT(rpcState.original_packet.get() != nullptr);
	auto findnodereq(check_and_cast<KadFindNodeReq *>(rpcState.original_packet.get()));

	if (rpcState.status == RPC_SUCCESS) {
		auto answer = safe_packet_cast<KadFindNodeRsp>(answer_);
		ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
		EV_TRACE << "received FindNodeRsp for " << findnodereq->getDstNodeID().shortStr();
		EV_TRACE << " from " << answer->getSource().shortStr() << " with " << answer->getContactArraySize() << " contacts" << endl;
		readRTableRsp(*answer);
	}

	// the following should also be executed in case we had an error (not RPC_SUCCESS)
	// try next via contact from RDS list of contact
	// if this was a rediscovery packet, check for its success
	if (findnodereq->getRediscovery() == true) {
		if (auto contactp= routingTable.getContact(findnodereq->getDstNodeID()); contactp != nullptr) {
			if (contactp->isRediscovering()) { // rediscovery not successful yet
				EV_TRACE << "Rediscovery not successful for contact " << *contactp << ", next try." << endl;
				// must trigger next rediscovery
				// send a new find node req
				// fanout set to one, because each failed rediscovery request
				// should only trigger a single new retry
				//ASSERT(contactp->getRDS().rediscoverQtype != RediscoveryState::rediscoverQ_type::undefined);
                                simtime_t rediscoveryWaitTime(
                                    (contactp->getRDS().rediscoverQtype == RediscoveryState::rediscoverQ_type::physicalneighbors) ? PNRediscoveryWaitTime :
				    (contactp->getRDS().rediscoverQtype == RediscoveryState::rediscoverQ_type::urgent) ? urgentRediscoveryWaitTime :
				    (contactp->getRDS().rediscoverQtype == RediscoveryState::rediscoverQ_type::regular) ? regularRediscoveryWaitTime : slowRediscoveryWaitTime);

				addOneShotTimer(SimTime(RediscoveryNextTryWaitTime,SIMTIME_MS), std::bind(&R2Kademlia::sendRediscovery, this, contactp->getKey(), 1, rediscoveryWaitTime));
				//sendRediscovery(contactp->getKey(), 1, rediscoverQ, rediscoveryWaitTime);
                        }
		}
	}
	return true;
}



bool R2Kademlia::handleRPCTimeOut(KadPacket_p answer_, RPCState& rpcState) {

	if (rpcState.status != RPC_TIMEOUT) return true;
	// answer_ will be nullptr and can be ignored

	// check for retransmissions to perform
	if (rpcState.retransmissions < rpcState.maxRetransmissions) {

		rpcState.status= RPC_WAITING;
		rpcState.retransmissions++;

		EV_TRACE << "performing retransmission of RPC: " << rpcState.retransmissions
			 << " of " << rpcState.maxRetransmissions << endl;

		// need original message again
		packet_p rpcCall= std::unique_ptr<packet_t>(rpcState.original_packet->dup());
		RPCtimeoutMsg* rpc_timeout_msg= new RPCtimeoutMsg(rpcState.nonce);
		rpcState.timeoutMsg= rpc_timeout_msg; // just a reference for cancelling
		// start timer for RPC timeout
		scheduleAt(simTime() + SimTime(rpcState.timeout_ms,SIMTIME_MS), rpc_timeout_msg);
		// send out
		sendPacket(std::move(rpcCall));

		return false;
	}
	// else: no retransmissions
	// nothing to do, RPC state will be deleted

	EV_TRACE << "RPC Timeout for msg: " << rpcState.original_packet->getId()
		 << " nonce:" << rpcState.nonce << endl;

	return true;
}


// this is called periodically for _all_ modules in Statistics::sample()
// This is not a subset, these statistics include all nodes!
void R2Kademlia::sampleStatistics (Statistics &stats) {
	// every value here will contribute to the average, min, max,
	// avgDiffQuot and quantiles, a histogram is also created

        // for link statistics we need to access the underlying node
        auto linkusagemap = node().getLinkUsageMap();
	for ( auto lu : linkusagemap) {
		stats.value("linkUsageMsgCount", lu.second.msgcount);
                stats.value("linkUsageReceivedBytes", lu.second.bytecount);
	}

        stats.value("forwardedPackets", forwardedPackets);
	stats.value("sentPackets", sentPackets);
	stats.value("receivedPackets", receivedPackets);
	stats.value("sentVolume", sentVolume);
	stats.value("receivedVolume", receivedVolume);
        stats.value("forwardedVolume", forwardedVolume);
	if (detailedKIRAMessageStats) {
		for (auto msgtype : msgStats) {
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_sentCount), msgtype.second.sentCount);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_forwardedCount), msgtype.second.forwardedCount);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_receivedCount), msgtype.second.receivedCount);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_droppedCount), msgtype.second.droppedCount);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_sentVolume), msgtype.second.sentVolume);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_forwardedVolume), msgtype.second.forwardedVolume);
			stats.value(msgStatsNames.getName(msgtype.first,msg_stats_subtype::msg_stat_receivedVolume), msgtype.second.receivedVolume);
		}
	}
	if (sendDataPackets) {
		stats.value("forwardedDataPackets", forwardedDataPackets);
		stats.value("sentDataPackets", sentDataPackets);
		stats.value("receivedDataPackets", receivedDataPackets);
	}
	stats.value("sentRouteUpdates", sentRouteUpdates);
	stats.value("receivedRouteUpdates", receivedRouteUpdates);
	stats.value("sentPathSetup", sentPathSetup);
	stats.value("receivedPathSetup", receivedPathSetup);
	stats.value("forwardedPathSetup", forwardedPathSetup);
	stats.value("routingTableContacts", routingTable.getNumContacts());
	stats.value("routingTableBuckets", routingTable.getNumBuckets());
	stats.value("routingTableUpdates", routingTable.getUpdateCount());
	stats.value("routingTableReplacements", routingTable.getReplacementCount());
	if (detailedRoutingTableStats) {
		stats.value("routingTablePathLength", routingTable.getAveragePathLength());
		auto [rt_stretch, no_faulty_paths] = routingTable.getAveragePathStretch();
                if (rt_stretch > 0) // it rt_stretch == 0 do not add the value, node has no valid contacts left
                  stats.value("routingTablePathStretch", rt_stretch);
		stats.value("routingTableFaultyPaths", no_faulty_paths);
		stats.value("routingTableContactsWithoutPN", routingTable.getNumContactsWithoutPNs());
		stats.value("routingTablePNContacts", routingTable.getNumContacts() - routingTable.getNumContactsWithoutPNs());
		stats.value("routingTableFailedPNs", routingTable.getFailedPNs().size());
	}
	unsigned long notifyFirstSegmentSourcesSize(0UL);
	std::for_each(notifyFirstSegmentSources.cbegin(), notifyFirstSegmentSources.cend(), [&](auto& entry){notifyFirstSegmentSourcesSize+= entry.second.size();});
	stats.value("notifyFirstSegmentSources",notifyFirstSegmentSourcesSize);
	stats.value("notifyFirstSegmentSourcesViaPID",notifyFirstSegmentSourcesReversePath.size());
	//stats.value("routingTableFoundEquivalentPaths", routingTable.getNumFoundEquivPaths());
	stats.value("fwdTableSize", fwdTablePathID.getNumEntries());
	stats.value("fwdTableValid", fwdTablePathID.getNumValidEntries());
	stats.value("fwdTableValidLocal", fwdTablePathID.getNumValidLocalEntries());
	stats.value("fwdTableForeign", fwdTablePathID.getNumForeignEntries());
	auto [numPrecalcEntries, numUsedPrecalcEntries]= fwdTablePathID.getNumUsedPrecalcEntries();
	stats.value("fwdTablePrecalc", numPrecalcEntries);
	stats.value("fwdTableUsedPrecalc", numUsedPrecalcEntries);
	stats.value("fwdTableUpdates", fwdTablePathID.getNumUpdates());
	stats.value("floodQueueMaxLength", FloodQueue.getMaxLength());
	stats.value("notifyQueueMaxLength", NotifyQueue.getMaxLength());
        if (getBoundProtocolPacketQueue(KAD_CTRL_PACKET_TYPE)) {
		stats.value("CPoverloadDiscard", getBoundProtocolPacketQueue(KAD_CTRL_PACKET_TYPE)->getOverloadDiscardedPackets());
                stats.value("CPmaxSojournTime", getBoundProtocolPacketQueue(KAD_CTRL_PACKET_TYPE)->getMaxSojournTime().dbl());
	}
    stats.value("floodPacketReplications", floodPacketReplications);
}


// gets called from KadObserver::recordStatistics()
// in finish() during sampling statistics from nodes
// (this is usually a reduced set, not all nodes!)
void R2Kademlia::recordScalars() {
	recordScalar("forwardedPackets", forwardedPackets);
	recordScalar("discardedPackets", discardedPackets);
	recordScalar("sentPackets", sentPackets);
	recordScalar("receivedPackets", receivedPackets);
	recordScalar("sentVolume", sentVolume);
	recordScalar("receivedVolume", receivedVolume);

	if (sendDataPackets) {
		recordScalar("forwardedDataPackets", forwardedDataPackets);
		recordScalar("droppedDataPackets", droppedDataPackets);
		recordScalar("sentDataPackets", sentDataPackets);
		recordScalar("receivedDataPackets", receivedDataPackets);
	}

	recordScalar("rediscoverySent", sentRediscovery);
	recordScalar("rediscoverySuccess", succeededRediscovery);
	recordScalar("rediscoveryFailed", droppedRediscovery);
	recordScalar("rediscoveryForwarded", forwardedRediscovery);
	recordScalar("rediscoveryRetries", retriesRediscovery);
	recordScalar("routingTableContacts", routingTable.getNumContacts());
	recordScalar("routingTableBuckets", routingTable.getNumBuckets());
	recordScalar("routingTablePathLength", routingTable.getAveragePathLength());
	auto [rt_stretch, no_faulty_paths] = routingTable.getAveragePathStretch();
	recordScalar("routingTablePathStretch", rt_stretch);
	recordScalar("routingTableFaultyPaths", no_faulty_paths);
	recordScalar("routingTableUpdates", routingTable.getUpdateCount());
	recordScalar("routingTableReplacements", routingTable.getReplacementCount());
	recordScalar("discoveredVicinitySize", vicinity.size());
	nodeid_t nid =  Node::getNode(this)->getNodeId();
	recordScalar("nodeId", nid);
	recordScalar("degree", getNetworkManager()->getTopology().getNodeDegree(nid));
	recordScalar("popularity", getKadObserver().getPopularity(myOverlayKey));
	// calulating pathpopularity is costly, only activate if really needed
        //recordScalar("pathpopularity", getKadObserver().getPathPopularity(myOverlayKey));
	unsigned long notifyFirstSegmentSourcesSize(0UL);
	std::for_each(notifyFirstSegmentSources.cbegin(), notifyFirstSegmentSources.cend(), [&](auto& entry){notifyFirstSegmentSourcesSize+= entry.second.size();});
	recordScalar("notifyFirstSegmentSources",notifyFirstSegmentSourcesSize);
	recordScalar("fwdTableSize", fwdTablePathID.getNumEntries());
	recordScalar("fwdTableValid", fwdTablePathID.getNumValidEntries());
	recordScalar("fwdTableValidLocal", fwdTablePathID.getNumValidLocalEntries());
	recordScalar("fwdTableForeign", fwdTablePathID.getNumForeignEntries());
	auto [numPrecalcEntries, numUsedPrecalcEntries]= fwdTablePathID.getNumUsedPrecalcEntries();
	recordScalar("fwdTablePrecalc", numPrecalcEntries);
	recordScalar("fwdTableUsedPrecalc", numUsedPrecalcEntries);

	recordScalar("floodPacketsReceived", floodPacketsReceived);
    recordScalar("floodPacketReplications", floodPacketReplications);
    recordScalar("floodPacketAvgReplications", floodPacketReplications / (double) floodPacketsReceived);
    recordScalar("floodPacketMaxReplications", floodPacketMaxReplications);

	// EV_TRACE << (shortenedlaterpaths * 100.0) / (double) laterpaths << "% "
	// 	 << shortenedlaterpaths << "/" << laterpaths << " of later paths shortened. "
	// 	 << "Viable PIDs: " << viablePIDpaths << "(" << (viablePIDpaths*100.0)/(double) laterpaths << "%)" << endl;
}

void R2Kademlia::recordRoutingTable() {
	// these submodules are a hack to record scalars for the contacts
	// they are only used for scalar recording
	cModuleType* modType = cModuleType::get("routingsim.routing.KIRA.Dummy");
	auto allContacts = routingTable.getAllContacts();
	std::string contactStr{"contact"};
	for (size_type i= 0; i < allContacts.size(); i++) {
		KadContact_p contact = allContacts[i];
		cModule* mod = modType->create((contactStr + std::to_string(i)).c_str(), this);
		mod->recordScalar("pathLength", contact->getPathLength()+1);
		mod->recordScalar("distance", getKadObserver().getPhysicalDistance(myOverlayKey, contact->getKey()));
		mod->recordScalar("commonPrefix", myOverlayKey.sharedPrefixLength(contact->getKey()));
	}
}

// -------------------- private helper methods --------------------

// this function modifies the source route in case a _valid_ contact to the destination
// is known. This should be safe even during rediscovery in case the notVia info was
// checked before calling this function.
void R2Kademlia::applyShortcut(KadPacket& packet, const OverlayKey& destination)
{
	auto knownContact= routingTable.getContact(destination);

	if (knownContact != nullptr)
	{ // final destination Contact is known, check for route shortcut
		if (knownContact->isValid())
		{
			// getProgress() is currently pointing to my own node ID
			kad_path path_of_interest= packet.getSrcPath(packet.getProgress()+1, packet.getRouteArraySize()-1);
			if (path_of_interest.size() > 1) // not much shortcut possible for only one hop
			{
				if (knownContact->getPathLength()+1 > path_of_interest.size())
					return;

				// TODO now check for cycles
				// replace residual path with my known (shorter) route
				EV_TRACE << "modifying sroute due to shortcut: old=" << packet.getSrcPath();

				kad_path newpath(knownContact->getPath());
				newpath.push_back(knownContact->getKey());
				packet.modRoute(newpath);

				EV_TRACE << " new=" << packet.getSrcPath() << endl;
			}
		}
	}
}


void R2Kademlia::fillRTableRsp(KadRTableRsp& packet, const KadContactList& contacts, bool includePaths)
{
	ASSERT(packet.getContactArraySize() == 0);
	ASSERT(packet.getPathArraySize() == 0);
	// in rare cases contact list may be empty
	if (contacts.size() > 0)
	{
		packet.setContactArraySize(contacts.size());
		packet.setNodeDegreeArraySize(contacts.size());
		if (includePaths) {
			packet.setPathArraySize(contacts.size());
			packet.setSeqNoArraySize(contacts.size());
			packet.setPathInfoAgeArraySize(contacts.size());
		}
		for (size_t i = 0; i < contacts.size(); i++) {
			packet.setContact(i, contacts[i]->getKey());
			if (includePaths) {
				packet.setPath(i, contacts[i]->getPath());
				packet.setSeqNo(i, contacts[i]->getStateSeqNo());
				packet.setPathInfoAge(i, contacts[i]->getPathAge());
				// path + seqno + age
				packet.addBitLength(contacts[i]->getPathLength()*OverlayKey::getLength()+32+sizeof(age_t)*8);
			}
			packet.setNodeDegree(i, contacts[i]->getNodeDegree());
			//ASSERT((stateseqno_t) contacts[i]->getNodeDegree() > contacts[i]->getStateSeqNo());
		}
		// contact list + node degree
		packet.addBitLength(contacts.size()*(OverlayKey::getLength()+16));
	}
}


void R2Kademlia::readRTableRsp(const KadRTableRsp& packet) {
	ASSERT(packet.getContactArraySize() == packet.getPathArraySize());
	kad_path viaPath = packet.reversePath(packet.getIntermediatePath());
	viaPath.push_back(packet.getSource());
	// should shorten path here already so that the in loop condition fits
	routingTable.shortenPath(viaPath,0);

	for (size_t i = 0; i < packet.getContactArraySize(); i++) {
		// TODO optimization: isPathInteresting
		// Examples: contact 0779b9 via path < ee2741 0de89c > + <->
		//           contact 06cc5e via path < ee2741 0de89c > + < d9c74f >
		//EV_TRACE << "readRTableRsp: contact " <<  packet.getContact(i).shortStr() << " via " << viaPath << " + " << packet.getPath(i) << endl;

		// check whether contact is not contained in path (avoid cycles)
		if ( find(viaPath.cbegin(), viaPath.cend(), packet.getContact(i)) != viaPath.cend()
		     or
		     find(packet.getPath(i).cbegin(), packet.getPath(i).cend(), packet.getContact(i)) != packet.getPath(i).cend() ) {
			//EV_TRACE << "skipping contact as it occurs already in path" <<  endl;
			continue;
		}

		// try to add contact or update path and/or contact meta data
		if ( not(routingTable.addPath(packet.getContact(i), packet.getPath(i), packet.getSeqNo(i), packet.getPathInfoAge(i), packet.getNodeDegree(i), viaPath)) ) {
			// NOTE addPath() returns false for non verified paths
			// even if contact was not interesting for RT, add it as PID entry if within vicinitydiscovery radius
			// the following condition is correct because the target contact
			// is not contained in (viaPath + packet.getPath(i))
			bool querynode(false);
			if ( packet.getContact(i)!= myOverlayKey and ((viaPath.size() + packet.getPath(i).size()) < nodeVicinityGraph::vicinityDiscoveryRadius) )
			{
				// for further processing we need a temporary contact object
				// since it was not put into the routingTable
				KadContact neighborhoodContact(myOverlayKey,
							       packet.getContact(i),
							       routingTable.concatViaPath(viaPath, packet.getPath(i)),
							       packet.getSeqNo(i));
				auto& parentNodeKey= neighborhoodContact.getPath().empty() ? myOverlayKey : neighborhoodContact.getPath().back();
				// try to add node to vicinity, but maybe already there
				auto [nodeAdded, linkAdded, vn]= vicinity.addNode(parentNodeKey, // parent node
								  neighborhoodContact.getKey(), // new child node
								  neighborhoodContact.getPathLength()+1); // distance of child node to us
				if (nodeAdded) {
					// newly put into vicinity, so get its ULNs (unconditionally, therefore store invalid seq no)
					querynode= true;
				}
				else {
					// newer sequence number for existing neighbor? reget ULNs
					if (vn != nullptr and vn->stateseqno < neighborhoodContact.getStateSeqNo())
						querynode= true;
				}

				// still room for one more hop in neighborhood exploration?
				// the same is done for new contacts in RT (see onNewContact)
				//bool expandDiscovery(viaPath.size() + packet.getPath(i).size() + 1  < nodeVicinityGraph::vicinityDiscoveryRadius);
				// add PID entry as this neighbor is in our vicinityDiscoveryRadius
				if (usePathIDs and (nodeAdded or linkAdded)) {
					auto pathtoadd(neighborhoodContact.getPath());
					pathtoadd.push_back(neighborhoodContact.getKey());
					fwdTablePathID.addPrecalcEntry(pathtoadd);
					vicinityPathIDCompletion(parentNodeKey, neighborhoodContact.getKey());
				}
			} // endif entry within vicinitydiscovery

			// only get further ULNs from node if strictly smaller than discovery radius
			// this should improve the routing table contacts
			if ( querynode and ((viaPath.size() + packet.getPath(i).size() + 1) < discoveryRadius) ) {
				KadContact neighborhoodContact(myOverlayKey,
							       packet.getContact(i),
							       routingTable.concatViaPath(viaPath, packet.getPath(i)),
							       packet.getSeqNo(i));

				scheduleGetULNReq(std::move(neighborhoodContact));
			}
		} // endif: new path not added
	} // end for each reported contact
}


// parameter times is used for counting broadcast packets like ULNHello
void R2Kademlia::countSent(KadPacket &packet, unsigned int times) {
	if (times == 1) {
		msgStats[packet.getType()].sentCount++;
		msgStats[packet.getType()].sentVolume += packet.getByteLength();
		sentVolume += packet.getByteLength();
		sentPackets++;
        } else {
		msgStats[packet.getType()].sentCount += times;
		msgStats[packet.getType()].sentVolume += packet.getByteLength()*times;
		sentVolume += packet.getByteLength()*times;
		sentPackets += times;
	}
}


void R2Kademlia::countFwd(KadPacket &packet) {
        if (not packet.isTracked()) {
		msgStats[packet.getType()].forwardedCount++;
		msgStats[packet.getType()].forwardedVolume += packet.getByteLength();
		forwardedVolume += packet.getByteLength();
                forwardedPackets++;
	}
}


void R2Kademlia::countReceived(KadPacket& packet) {
	//EV_TRACE << "countReceived: tracked=" << packet.isTracked() << endl;
        // reachability test packets should not be counted
	if (not packet.isTracked()) {
		msgStats[packet.getType()].receivedCount++;
		msgStats[packet.getType()].receivedVolume += packet.getByteLength();
		receivedVolume += packet.getByteLength();
		receivedPackets++;
        }
	else {
		// registerPacketDelivered will also count first packet stretch
		getKadObserver().registerPacketDelivered(packet);
		ASSERT(packet.getControlInfo() != nullptr);
		getStatistics().deliveredMessage(
			check_and_cast<cMessage *>(packet.getControlInfo()),
			packet.getProgress());
	}
}


void R2Kademlia::countDropped(KadPacket& packet, const OverlayKey& target, uint8_t reason) {
	discardedPackets++; // note that this is different from droppedPackets in KadObserver
	if (packet.isTracked()) {
          EV_TRACE << "+DROPPED+ nonce:" << packet.getNonce()
                   << " reason:" << (unsigned int)reason << endl;
        } else {
		msgStats[packet.getType()].droppedCount++;
	}

	getKadObserver().registerPacketDropped(packet, myOverlayKey, target, reason);
	nodeid_t nid = undefined_node;
	if (getKadObserver().exists(target))
		nid = getKadObserver().getNodeId(target);
	if(packet.getControlInfo() == nullptr) return;
	getStatistics().dropMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()), nid
	);
}


void R2Kademlia::returnToSender(uint8_t errorType, KadPacket_p packet, const OverlayKey& failedNextHopNodeID) {
	KadError_p answer = makeAnswer<KadError>(*packet);
	answer->setErrorType(errorType);

	EV_TRACE << "returnToSender() - error: " << (unsigned int) errorType << " - return path:";
	for (const auto& c : answer->getSrcPath())
		EV_TRACE << ' ' << c.shortStr();
	EV_TRACE << endl;
	answer->setDstNodeID(packet->getSource());
	answer->setFailedNextHopNodeID(failedNextHopNodeID);
	answer->setOrigDstNodeID(packet->getTarget());
	answer->encapsulate(packet.release());
	if (answer->hasRemainingHops())
		sendPacket(std::move(answer));
	else // we sent this packet!
		handleAnswer(std::move(answer));
}


// note: the ppq is empty afterwards as the queue has no iterator
// void
// R2Kademlia::inspectQueue(ProtocolPacketQueue& ppq) const {
//   unsigned int fwd_packets = 0UL;
//   unsigned int local_packets = 0UL;
//   unsigned int overlay_packets = 0UL;
//   std::queue< std::pair<simtime_t, NodePkt_p> >& tmpq= ppq.getRawQueue();
//   while (not tmpq.empty()) {
//     NodePkt* np= tmpq.front().second.get();
//     if (np) {
//       const auto pp= dynamic_cast<KadPacket*>(np->getEncapsulatedPacket());
//       if (pp) {
//         EV_TRACE << "Queued Message: " << pp->getName();
//         if (pp->hasRemainingHops()) {
//           fwd_packets++;
// 	  EV_TRACE << " fwd src route" << endl;
//         }
//         else {
//           if (pp->getTarget() == myOverlayKey) {
// 	    EV_TRACE << " local" << endl;
//             local_packets++;
//           }
//           else {
// 	    EV_TRACE << " fwd overlay" << endl;
//             overlay_packets++;
// 	  }
//         }
//       }
//     }
//     tmpq.pop();
//   }
//   EV_TRACE << "inspect queue=" << " fwd: " << fwd_packets << " local: " << local_packets << " overlay: " << overlay_packets << endl;
// }



} // namespace
