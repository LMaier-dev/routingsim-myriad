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

#include <BaseTypes.h>

#include "Statistics.h"
#include "TrickleTimer.h"

#include "R2Kademlia.h"
#include "KadRoutingTable.h"
#include "KadObserver.h"

#include "FwdTable.h"

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;

// callback function type
using cbfunc_t = std::function<void(short timerId)>;

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
	OverlayKey::setKeyLength(OverlayKeyLength);
	myOverlayKey = OverlayKey::random(getRNG(0));
	fwdTablePathID.setOwnNodeID(&myOverlayKey);
	vicinity.setRoot(myOverlayKey);
	getKadObserver().registerUp(myOverlayKey, node().getNodeId());
	bind(KAD_CTRL_PACKET_TYPE);
	bind(KAD_DATA_PACKET_TYPE);

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
	if (sendDataPackets and not usePathIDs)
	{
		EV_WARN << "usePathIDs must be set if sendDataPackets is enabled, changed to usePathIDs=on" << endl;
		usePathIDs= true;
	}
	probeStrategy = (uint) par("probeStrategy").intValue();
	detailedRoutingTableStats = par("detailedRoutingTableStats").boolValue();
	urgentRediscoveryWaitTime = (simtime_t) (par("urgentRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	regularRediscoveryWaitTime = (simtime_t) (par("regularRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	slowRediscoveryWaitTime = (simtime_t) (par("slowRediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	updateWaitTime = (simtime_t) (par("updateWaitTime").doubleValue() * 0.001);  // given in ms
	updateWaitTimeFast = (simtime_t) (par("updateWaitTimeFast").doubleValue() * 0.001);
	pathSetupWaitTime = (simtime_t) (par("pathSetupWaitTime").doubleValue() * 0.001);  // given in ms
	rediscoveryFanOutFactor = (uint) par("rediscoveryFanOutFactor").intValue();
	rediscoveryMaxNotification = (uint) par("rediscoveryMaxNotification").intValue();
	updateMaxNotification = (uint) par("updateMaxNotification").intValue();
	updateNotifyStrategy = (uint) par("updateNotifyStrategy").intValue();
	updatePropagateStrategy = (uint) par("updatePropagateStrategy").intValue();
	failureFocussedProbing = par("failureFocussedProbing").boolValue();

	// register this module for statistics
	getStatistics().addSample(this);
	simtime_t probeInterval = (simtime_t) par("probeInterval").doubleValue();
	simtime_t exploreInterval = (simtime_t) par("exploreInterval").doubleValue();

	// doProbeSelf is required for joining (will do exponential backoff)
	// calculation for first timer is: initialOffset - duration/2 + random([0..duration])
	// start after 100ms, use 250ms, 500ms, 1s, 2s, 4s, 8s, 16s, 32s, 64s, 128s, 256s, 300s as intervals
	probeTimerId= addRandomizedTimer(SimTime(225,SIMTIME_MS), SimTime(250,SIMTIME_MS), std::bind(&R2Kademlia::doProbeSelf, this, ph::_1));

	// regularly probe for closest (overlay/ID-wise)  neighbors
	addRandomizedTimer(std::max(probeInterval, SimTime(1000,SIMTIME_MS)), probeInterval*2, std::bind(&R2Kademlia::doProbeNeighbors, this, ph::_1));
	// regularly probe for non PN contacts to test path liveliness
	addRandomizedTimer(std::max(probeInterval/2, SimTime(1000,SIMTIME_MS)), probeInterval, std::bind(&R2Kademlia::doProbeContacts, this, ph::_1));
	// regularly send FindNodeReq to _randomly_ chosen IDs, i.e., key-IDs but not node-IDs
	// the advantage is to discover new and better contacts and/or paths
	addRandomizedTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doProbe, this, ph::_1));

	// Reachability Test: check whether other _existing_ nodes can be reached (uses KadObserver)
	addRandomizedTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doReachabilityTest, this, ph::_1));

	// For path exploration
	if (exploreInterval > 0)
		addRandomizedTimer(exploreInterval*1.5, exploreInterval, std::bind(&R2Kademlia::doSendExplorePathPacket, this, ph::_1));

	// resend Hello packets to keep state sequence numbers in synch
	addRandomizedTimer(SimTime(225,SIMTIME_MS), SimTime(250,SIMTIME_MS), std::bind(&R2Kademlia::doSendHello, this, ph::_1));

	if (usePathIDs) {
		// if PathRefreshThreshold is 20s, refresh is scheduled between [10s,30s]
		addRandomizedTimer(SimTime(PathRefreshThreshold/2,SIMTIME_MS), SimTime(PathRefreshThreshold,SIMTIME_MS), std::bind(&R2Kademlia::doRefreshSetupPaths, this, ph::_1));
		// garbageCollection would happen after 60s
		addFixedTimer(SimTime(3*PathRefreshThreshold,SIMTIME_MS), SimTime(3*PathRefreshThreshold,SIMTIME_MS), std::bind(&R2Kademlia::doGarbageCollectPathIDs, this, ph::_1));
	}

	if (sendDataPackets)
	{
	  EV << "will send data packets after " << getSimulation()->getWarmupPeriod() << endl;
	  addRandomizedTimer(getSimulation()->getWarmupPeriod()+probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doDataProbe, this, ph::_1));
	}

	// expose the following variables in GUI for inspection
	WATCH(myOverlayKey);
	WATCH(lsseqno);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);
	WATCH_OBJ(fwdTablePathID);

	// the following is not necessary, because onConnect will do this
	//sendToNeighbors(makeHelloPacket());
}


// link comes up (treat like a link layer indication "link established"), immediately send a PNHello message
void R2Kademlia::onConnect( nodeid_t nid ) {

	// send a hello packet after some random delay
	addOneShotTimer(SimTime(uniform(1000,3000),SIMTIME_US), [this,nid]() {
		auto packet(makeHelloPacket());
		sentVolume+= packet->getByteLength();
		sendToNode(nid, std::move(packet));
		sentPackets++;
	} );

}


// link is down (treat like link layer indication "link down")
void R2Kademlia::onDisconnect( nodeid_t nid ) {
	//EV_TRACE << "onDisconnect " << nid << endl;
	// if this node is taken down, the packets arrive after the reset
	if (neighborTable.size() == 0)
		return;

	OverlayKey key = neighborTable.getOverlayKey(nid);
	ASSERT(not key.isUnspecified());
	ASSERT(neighborTable.getNodeId(key) == nid );

	// change in physical neighborhood increases the Sequence Number
	incLSSeqNo();
	neighborTable.removeNeighbor(nid);
	ASSERT(neighborTable.getNodeId(key) == undefined_node);
	// set contact invalid (was routingTable.removeContact(key))
	if (not routingTable.setContactInvalid(key)) {
		// this should only happen for nodes with very large degree
		EV_ERROR << "missing contact " << nid << " " << key << " on node " << node().getNodeId() << endl;
		return;
	}

	// Try to rediscover that contact
	// after grace period (otherwise more links may fail immediately after this)
	if (routingTable.isInLastBucket(key))
		scheduleUrgentRediscovery(key);
	else
		scheduleRegularRediscovery(key);


	// contacts from routing table with this neighbor as next hop are not usable until rediscovered
	routingTable.invalidateContactsPN(key);

	// send UpdateRouteReq w/ failed link to ID-wise closest Nodes
	// we can rely on routingTable.getContact(key) != nullptr, because of earlier setContactInvalid() call
	scheduleUpdate(*(routingTable.getContact(key)), KAD_Unreachable);
}


// called on reception of a NodePkt for this protocol
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

	ASSERT(pkt->getEncapsulatedPacket() != nullptr);

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

	// other packets should include the key in the source route
	packet->advanceProgress(); // increase hop count
	if (packet->getCurrentHop() != myOverlayKey) {
		EV_ERROR << "dropping packet, current hop in source route does not match!" << endl;
		//TODO how does this happen?
		countDropped(*packet, packet->getTarget(), erroneousSourceRoute);
		return;
	}

	// must process PNDiscReq to set the neighbor table entry first
	if (handleIfFitting<KadPNDiscReq>(packet))
		return;

	// check for presence of carried "not via" information
	// and evaluate it (invalidate affected contacts)
	processNotViaInformation(*packet);

	if (packet->hasRemainingHops()) {
		// we are not the recipient

		//TODO ERROR OVERHEARING, more overhearing
		// special actions while forwarding packets
		switch (packet->getType())
		{

			case KAD_PathSetupReq:
			{
				// as intermediate node I need to check to path ID installation
				// returns true if path was installed: forward to next hop only if
				// hops left > vicinityDiscoveryRadius+1
				bool return_answer(false);
				if ( checkForPathInstallation(*packet) ) {
					// PID was locally installed
					// send response if hops to go <= vicinityDiscoveryRadius+1
					if (packet->getRouteArraySize()-packet->getProgress()
					    <= nodeVicinityGraph::vicinityDiscoveryRadius+2)
						return_answer= true;
				}
				else { // send back response if path ID already present/installed
					EV_TRACE << "KadPathSetupReq: PID for spath " << packet->getSrcPath(packet->getProgress())
						 << " already present" << endl;
					return_answer= true;
				}

				if (return_answer) {
					// check for path improvements since this returns early here
					if (pathSnooping) {
						kad_path path_of_interest(packet->reversePath(packet->getSrcPath(1, packet->getProgress())));
						if (path_of_interest.size() > 0)
						{
							routingTable.addPath(packet->getSource(), path_of_interest, packet->getStateSeqNo(), 0);
						}
					}
					const KadPathSetupReq* setupreq = static_cast<const KadPathSetupReq *>(packet.get());
					sendPacket(makeAnswerKadPathSetupRsp(*setupreq));
					return; // this will delete the original packet
				}
				forwardedPathSetup++;
			}
			break;

			case KAD_PathTeardownReq:
			{
				// as intermediate node I need to check to path ID teardown
				checkForPathTeardown(*packet);

				if (packet->getRouteArraySize()-packet->getProgress() <= nodeVicinityGraph::vicinityDiscoveryRadius+2) {
					EV_TRACE << "KadPathTeardown: terminated here, the rest is precomputed" << endl;
					return;  // this will delete the original packet
				}

				forwardedPathTeardown++;
			}
			break;

			case KAD_PathSetupRsp:
			{
				EV_TRACE << "forwarding PathSetupRsp, origin spath: "
					 << packet->reversePath(packet->getSrcPath()) << endl;
			}
			break;

			case KAD_ProbeReq: // this is for path probing
			{
				const KadProbeReq* probereq= dynamic_cast<const KadProbeReq*>(packet.get());
				// usually the following is not necessary, but sometimes
				// PathSetup was scheduled but not performed and a newer seq of a longer
				// path start a path check, resulting in the ProbeReq
				if (usePathIDs and probereq->getPathCheck()) {
					if (checkForPathInstallation(*packet))
						EV_TRACE << "POST Installed PathID by ProbeReq PathCheck" << endl;
				}
			}
			break;

			case KAD_UpdateRouteReq: // check for unreachable info
				evalUpdateReq(static_cast<const KadUpdateRouteReq*>(packet.get()));
				break;

			case KAD_QueryRouteRsp: // check for interesting contacts
				evalQueryRouteRsp(static_cast<const KadQueryRouteRsp*>(packet.get()));
				break;

			// case KAD_Error: // TODO check for contact invalidation due to SegmentFailure
			//
			// break;

		} // end switch

		if (pathSnooping) {
			// aka PATH OVERHEARING
			//TODO optimization use isPathInteresting

			// one can only trust the so far traversed path to work and exist
			// this is especially important for rediscovery messages: they may use still somewhat obsolete path information
			// also: avoid learning about supposed physical Neighbors
			kad_path path_of_interest(packet->reversePath(packet->getSrcPath(1, packet->getProgress())));
			if (path_of_interest.size() > 0)
			{
				routingTable.addPath(packet->getSource(), path_of_interest, packet->getStateSeqNo(), 0);
			}

			// NOTE: the rest of the source routing path should not be used for improvement
			// we will probably get the path back in the response message
		}

		forwardPacket(std::move(packet), std::move(pkt));
		return;
	}
	else { // no hops left, packet is addressed to me
		packet->setControlInfo(pkt.release());
		
		// learn route from packet
		//TODO optimization use isPathInteresting


		// for packets from physical neighbors: check need for resynchronization:
		// in case the packet indicates a newer state seq no, resynch with neighbor
		if (packet->getType() != KAD_PNDiscRsp and neighborTable.isNeighbor(packet->getSource()))
			checkForResync(packet->getSource(), packet->getStateSeqNo());

		// evaluate return path for routing table (packet came to us this way, so path was recently working)
		routingTable.addPath(packet->getSource(), packet->reversePath(packet->getIntermediatePath()), packet->getStateSeqNo(), 0);

		// these types may forward packets, so they count themselves
		if (handleIfFitting<KadFindNodeReq>(packet)) return;
		if (handleIfFitting<KadUpdateRouteReq>(packet)) return;
		// these types never forwardpackets
		countReceived(*packet);
		if (handleIfFitting<KadQueryRouteReq>(packet)) return;
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


// everything starts here: discovering a neighbor by seeing a PNHello packet
// in case the link was temporarily down: rediscover neighbor
void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "received hello packet from " << packet->getSource().shortStr() << " SSeq#" << packet->getStateSeqNo() << endl;

	receivedPackets++;
	receivedVolume+= packet->getByteLength();

	// either completely new PN or reseen after link repair
	if (neighborTable.addNeighbor(packet->getSourceId(), packet->getSource())) {
		// physical neighborhood changed -> increase local state variable
		incLSSeqNo();

		//EV_TRACE << "added to neighbor table: "<< neighborTable << endl;

		// PNDiscovery should be sent before any further messages to avoid missing neighbors
		// send back PNDiscReq to get contacts from physical neighbor
		KadContact neighborcontact(packet->getSource(), kad_path());

		// try to add neighbor to routing table
		if (routingTable.addPath(packet->getSource(), kad_path(), packet->getStateSeqNo(), 0))
		{
			EV_TRACE << "adding neighbor successful, its SSeq#" << packet->getStateSeqNo() << endl;
		}
		else
			EV_TRACE << "adding neighbor failed (maybe was already there)" << endl;

		// this node can be added safely to the vicinity now,
		// because PNDiscReq may be sent quite some time later
		vicinity.addNode(myOverlayKey, // parent
				 neighborcontact.getKey(), // new child
				 neighborcontact.getStateSeqNo(),
				 1);

		// fwdTablePathID.addEntry() and vicinityPathIDCompletion() are called by onNewContact

		// send PN Discovery Request immediately if I have the lower (NodeID % 32)
		// otherwise wait some time, probably sending a PNDiscReq is obsolete
		if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource()));
		    contact_p != nullptr) {

			// use this deterministic strategy to save redundant PNDiscReq/Rsp exchanges
			// do not use full ID, otherwise large IDs will always "loose", use mod 2^32 comparison
			// small collision chance: but just in case, full nodeID will be a tie breaker
			uint32_t ownModID(myOverlayKey.getBitRange(0,32));
			uint32_t otherModID(contact_p->getKey().getBitRange(0,32));
			uint32_t delta((uint32_t) (otherModID-ownModID));

			if ( delta < 0x80000000UL or
			     ((delta==0 or delta==0x80000000UL) and myOverlayKey < contact_p->getKey())
			   ) {
				sendDiscoveryReq(contact_p);
			}
			else // schedule PNDiscReq for later
				reSyncPN(*contact_p, packet->getStateSeqNo());
		}
	}
	else {
		EV_TRACE << "check for resync" << endl;
		// already known neighbor, just check for seqno synchronization
		checkForResync(packet->getSource(), packet->getStateSeqNo());

		return;
	}

	//EV_TRACE << "handlePacket: "<< static_cast<void*>(packet.get()) << endl;
}


void R2Kademlia::handlePacket(KadAnswer_p packet) {
	EV_DEBUG << "received " << packet->getName() << "(for nonce=" << packet->getNonce() << ") from " << packet->getSource().shortStr() << endl;
	if (packet->getType() == KAD_Error) {
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
		// note that handleAnswer() will be called  afterwards for processing the RPC result
	}
	else if (packet->getType() == KAD_FindNodeRsp) {

		// update StateSeqNo and lastseen of origin contact
		if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
			// it might be the case that packet was reordered by using a different path than an earlier message
			if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
				contact_p->setStateSeqNo(packet->getStateSeqNo());
				contact_p->updateLastSeen();
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
		if (routingTable.shortenPath(returnpath))
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


/// note: this method is also called when packet originate from this node
void R2Kademlia::forwardPacket(KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_PNHello);
	ASSERT(packet->hasRemainingHops());
	ASSERT(packet->getControlInfo() == nullptr);

	bool packetorigin(packet->getProgress() == 0);
	// TODO Check for already present NotVia() => loop
	// TODO Check for presence of own ID in so far processed path (maybe as overlay hop)
	
	//TODO optimization: potential shortcut - we check if the target is a already neighbor
	// only allowed for exact and not strict routing

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

		if (not packetorigin) {
			forwardedPackets++;
			//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << " delay:" << delay << endl;
			forwardedVolume+= packet->getByteLength();
		}

		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt), delay);

		return;
	}
	else { // next hop not available
               // for Routing Updates and Rediscovery messages try to use overlay-based forwarding
		EV_TRACE << "forwardPacket(): next hop " << nextHop.shortStr() << " unavailable" << endl;

		const OverlayKey* finaldestination(nullptr);
		KadUpdateRouteReq* updatepacket(nullptr);
		KadFindNodeReq* findnodepacket(nullptr);
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
			routingTable.simplifyPath(newpath, nullptr);
			EV_TRACE << "forwardPacket(): reroute PN, contact " << *contact << endl;
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

					if (not packetorigin) {
						forwardedPackets++;
						//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << endl;
						forwardedVolume+= packet->getByteLength();
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
			EV_TRACE << "forwardPacket(): trying to reroute to final destination " << *finaldestination << endl;
			// add not via information before forwarding
			if (  updatepacket != nullptr or findnodepacket != nullptr )
			{
				packet->addNotViaLink({nextHop,myOverlayKey}, 0);
			}

			// try to reroute
			reroutePacket(*finaldestination, std::move(packet), std::move(nodePkt), delay);
			return;
		}
	} // end else (no next hop found)

	EV_ERROR << "forwardPacket(): could not find next hop!" << endl;
	onMissingNeighbor(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::reroutePacket(const OverlayKey& finaldest, KadPacket_p packet, NodePkt_p nodePkt, simtime_t delay)
{
	std::list<std::pair<link_t,age_t>> notvialist;
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
		if (contact.isInvalid() or contact.getKey().isUnspecified())
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
				if (not packetorigin) {
					forwardedPackets++;
					//EV_TRACE << "forwardPacket(): Node" << node().getNodeId() << " " << packet->getName() << " nonce=" << packet->getNonce() << " progress:" << packet->getProgress() << " route:" << packet->getSrcPath() << endl;
					forwardedVolume+= packet->getByteLength();
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

	// if we get here: bad luck
	onMissingNeighbor(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::resendPacket(KadPacket_p packet) {
	getKadObserver().registerPacketOverlayHop(*packet);
	ASSERT(packet->getControlInfo() != nullptr);
	NodePkt_p nodePkt = NodePkt_p(check_and_cast<NodePkt *>(packet->removeControlInfo()));
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
	forwardPacket(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::sendPacket(KadPacket_p packet, simtime_t delay) {
	ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);

	if (not packet->hasRemainingHops() or packet->getTarget() == myOverlayKey) {
		EV_ERROR << "sendPacket: not sending packet destined to myself" << endl;
		return;
	}
	auto packetsize(packet->getByteLength());
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
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
	sentPackets++;
	sentVolume+= packetsize;
	forwardPacket(std::move(packet), std::move(nodePkt), delay);
}


/**
 * chooseNextHop performs proximity routing:
 * for contacts with the same XOR distance to target it selects the one with the shortest path
 * Assumption is that the contact list is sorted by XOR distance to target
**/
KadContactList::const_iterator
R2Kademlia::chooseNextHop(const KadContactList& contacts, const OverlayKey& target, KadContactList::const_iterator startit) {

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

				if ((myOverlayKey ^ target).log_2() == dprefix) {
					// if we have no prefix progress,
					// we take the closest contact by XOR, which is this one (assumption of XOR-sorted contacts)
					return bestchoice_it;
				}
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

				// stop at contact that is further away
				if (((*it)->getKey() ^ target).log_2() > dprefix)
					break;

				// contact is XOR-wise at the same distance (dprefix)
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

		if (not contacts.empty() and startit != contacts.end() and *startit != nullptr) {
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


// Path Probing: sends a ProbeReq to check for validity of an existing path
void R2Kademlia::sendPathProbe(const OverlayKey& destinationNodeID) {
	KadContact_p contact(routingTable.getContact(destinationNodeID));
	// must be a valid contact
	if ( contact == nullptr or contact->isInvalid() ) {
		EV_TRACE << "not sending ProbeReq from " << myOverlayKey.shortStr() << " to " << destinationNodeID.shortStr() << " because of invalid contact" << endl;
		return;
	}

	EV_TRACE << "sending ProbeReq from " << myOverlayKey.shortStr() << " to " << destinationNodeID.shortStr() << endl;
	auto probereq= makePacket<KadProbeReq>(contact, "KadProbeReq");
	probereq->setPathCheck(true);
	sendRpcCall(std::move(probereq),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2));
}


// sends back an answer for a path probe
void R2Kademlia::handlePacket(std::unique_ptr<KadProbeReq> packet) {
	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
			contact_p->setStateSeqNo(packet->getStateSeqNo());
			contact_p->updateLastSeen();
		}
		else
			EV_WARN << "State sequence number of message smaller (" << packet->getStateSeqNo() << ") than for contact (" << contact_p->getStateSeqNo() << ")" << endl;

	}

	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}


bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(probeRsp->getSource())); contact_p != nullptr) {
		// usually this should not be necessary as a path update is done before
		if (contact_p->getStateSeqNo() <= probeRsp->getStateSeqNo()) {
			contact_p->setStateSeqNo(probeRsp->getStateSeqNo());
			contact_p->updateLastSeen();
			contact_p->updateLastPathRefresh();
		}
		else
			EV_WARN << "State sequence number of message smaller (" << probeRsp->getStateSeqNo() << ") than for contact (" << contact_p->getStateSeqNo() << ")" << endl;
	}

	return true;
}



// Request Physical Neighbors from contact
// it seems crucial for slightly better stretch that this happens without any further delay
void R2Kademlia::sendGetPhysicalNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request (PN) to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	req->setOverlayNeighbors(false);

	// we rate limit the requests but only after 2s of simulated time to get better initial stretch values
	simtime_t delay(0);

	static const simtime_t mindelay(SimTime(10000,SIMTIME_US));
	if  ( simTime() < LastPNQueryRouteReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastPNQueryRouteReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastPNQueryRouteReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastPNQueryRouteReq+mindelay-simTime();
		}
	}

	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		delay
	);
	LastPNQueryRouteReq= simTime()+delay;

	// update vicinity to avoid future redundant requests
	bool nodeOrLinkAdded= vicinity.addNode(contact->getPath().empty() ? myOverlayKey : contact->getPath().back(), // parent
					       contact->getKey(), // new child
					       contact->getStateSeqNo(),
					       contact->getPath().size()+1);
	if (usePathIDs and nodeOrLinkAdded) {
		fwdTablePathID.addPrecalcEntry(contact);
		vicinityPathIDCompletion(*contact);
	}
}


// Request k nearest Overlay Neighbors from contact 
void R2Kademlia::sendGetOverlayNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request (ON) to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(0); // no physical neighbors
	req->setOverlayNeighborsSource(true); // get known overlay neighbors for my ID
	req->setContactsOnly(false); // get also the routes

	// we rate limit the requests
	simtime_t delay(0);
	static const simtime_t mindelay(SimTime(20000,SIMTIME_US));
	if  ( simTime() < LastONQueryRouteReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastONQueryRouteReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastONQueryRouteReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastONQueryRouteReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		delay
	);
	LastONQueryRouteReq= simTime()+delay;
}


// Send PNDiscReq
void R2Kademlia::sendDiscoveryReq(KadContact_p contact)
{
	ASSERT(contact != nullptr);
	EV_TRACE << "sending PNDiscReq to " << *contact << " mySSeq#: " << getLSSeqNo() << endl;
	auto req = makePacket<KadPNDiscReq>(contact, "KadPNDiscReq");
	req->setSourceId(node().getNodeId());
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	// send my own physical neighbors
	if (std::vector<OverlayKey> neighbors = neighborTable.getNeighbors(); neighbors.size() > 0) {
		req->setPhyNeighborArraySize(neighbors.size());
		req->setPhyNeighborSeqNoArraySize(neighbors.size());
		req->setPhyNeighborAgeArraySize(neighbors.size());
		for (uint i = 0; i < neighbors.size(); i++) {
			req->setPhyNeighbor(i, neighbors[i]);
			if (routingTable.getContact(neighbors[i]) != nullptr) {
				req->setPhyNeighborSeqNo(i, routingTable.getContact(neighbors[i])->getStateSeqNo());
				req->setPhyNeighborAge(i,routingTable.getContact(neighbors[i])->getAge());
			}
			else
				EV_ERROR << "Contact in RT for neighbor " << neighbors[i] << " not found" << endl; 
		}
	}
	//req->setHopLimit(1);
	// we rate limit the requests
	simtime_t delay(0);

	static const simtime_t mindelay(SimTime(10000,SIMTIME_US));
	if  ( simTime() < LastPNDiscReq ) {
		// last was scheduled in future, schedule it mindelay behind that
		delay= LastPNDiscReq-simTime()+mindelay;
	}
	else { // last send in past
		if (simTime()-LastPNDiscReq < mindelay) {
			// we are too early, so delay until mindelay is fulfilled
			delay= LastPNDiscReq+mindelay-simTime();
		}
		// otherwise no extra delay necessary
	}

	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2),
		delay
	);
	LastPNDiscReq= simTime()+delay;
	// add physical neighbor to my vicinity
	//commented out, because Hello should have already done so
	//vicinity.addNode(myOverlayKey, contact->getKey(), contact->getStateSeqNo(), 1);
}


// handle PNDiscReq
void R2Kademlia::handlePacket(std::unique_ptr<KadPNDiscReq> packet) {
	EV_TRACE << "received PNDiscReq from " << packet->getSource().shortStr()
		 << " SSeq#" <<  packet->getStateSeqNo()
		 << " w/ " << packet->getPhyNeighborArraySize() << " neighbors" << endl;

	receivedPackets++;
	receivedVolume+= packet->getByteLength();

	// since this is physical neighbor add it to our neighbortable
	if (neighborTable.addNeighbor(packet->getSourceId(), packet->getSource())) {
		// new neighbor -> increase sequence number
		incLSSeqNo();
	}

	// add physical neighbor to my vicinity
	bool node_added= vicinity.addNode(myOverlayKey, packet->getSource(), packet->getStateSeqNo(), 1);
	KadContact neighborcontact(packet->getSource(), kad_path());
	if (usePathIDs and node_added) {
		fwdTablePathID.addPrecalcEntry(&neighborcontact);
		vicinityPathIDCompletion(neighborcontact);
	}

	// check for physical neighbors sent with
	kad_path path{packet->getSource()};
	if (packet->getPhyNeighborArraySize() > 0) {
		for (uint i = 0; i < packet->getPhyNeighborArraySize(); i++) {

			routingTable.addPath(packet->getPhyNeighbor(i), path,
					     packet->getPhyNeighborSeqNo(i),
					     packet->getPhyNeighborAge(i));
		}
	}

	EV_TRACE << "neighbor table: "<< neighborTable << endl;

	// add physical contact to routing table or update sequence number
	routingTable.addPath(packet->getSource(), kad_path(), packet->getStateSeqNo(), 0);

	// update lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		contact_p->updateLastSeen();
		// if we got a list of PN, set PNsyncSeqNO to avoid too frequent resync
		if (packet->getPhyNeighborArraySize() > 0) {
			if (packet->getStateSeqNo() > contact_p->getSyncPNSeqNo())
				contact_p->setSyncPNSeqNo(packet->getStateSeqNo());
		}
		else {
			// if no PN list given we may want to resync
			checkForResync(packet->getSource(), packet->getStateSeqNo());
		}
	}

	// TODO currently only getting physical neighbors is implemented
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadPNDiscRsp>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	response->setContactArraySize(neighbors.size());
	response->setPathArraySize(neighbors.size());
	response->setSeqNoArraySize(neighbors.size());
	response->setPathInfoAgeArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++) {
		response->setContact(i, neighbors[i]);
		response->setSeqNo(i, routingTable.getContact(neighbors[i])->getStateSeqNo());
		response->setPathInfoAge(i,routingTable.getContact(neighbors[i])->getAge());
	}
	sendPacket(std::move(response));
}



void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet)
{
	EV_TRACE << "received KadQueryRouteReq from " << packet->getSource().shortStr()
		 << " SSeq#" <<  packet->getStateSeqNo() << endl;

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		if (contact_p->getStateSeqNo() <= packet->getStateSeqNo()) {
			contact_p->setStateSeqNo(packet->getStateSeqNo());
			contact_p->updateLastSeen();
		}
		// EV_TRACE << "contact found" << endl;
	}

	//  packet->getContactsOnly() not implemented yet
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadQueryRouteRsp>(*packet);
	if (packet->getNeighborhood() == 1)
	{
		std::vector<OverlayKey> neighbors(neighborTable.getNeighbors());
		response->setContactArraySize(neighbors.size());
		response->setPathArraySize(neighbors.size());
		response->setSeqNoArraySize(neighbors.size());
		response->setPathInfoAgeArraySize(neighbors.size());
		for (uint i = 0; i < neighbors.size(); i++) {
			if (routingTable.getContact(neighbors[i]) != nullptr)
			{
				response->setContact(i, neighbors[i]);
				response->setSeqNo(i, routingTable.getContact(neighbors[i])->getStateSeqNo());
				response->setPathInfoAge(i,routingTable.getContact(neighbors[i])->getAge());
			}
			else
				EV_ERROR << "Physical neighbor " << neighbors[i].shortStr() << " not present in routing table!" << endl;
		}
	}
	else
	if (packet->getNeighborhood() == 63) // means to get full routing table
	{
		KadContactList allcontacts= routingTable.getAllContacts();
		response->setContactArraySize(allcontacts.size());
		response->setPathArraySize(allcontacts.size());
		response->setSeqNoArraySize(allcontacts.size());
		response->setPathInfoAgeArraySize(allcontacts.size());
		for (uint i = 0; i < allcontacts.size(); i++)
		{
			response->setContact(i, allcontacts[i]->getKey());
			response->setPath(i, allcontacts[i]->getPath());
			response->setSeqNo(i, allcontacts[i]->getStateSeqNo());
			response->setPathInfoAge(i, allcontacts[i]->getAge());
		}
	}

	if (packet->getOverlayNeighbors() or packet->getOverlayNeighborsSource())
	{
		KadContactList ovlyneighbors= routingTable.getClosestContacts(packet->getOverlayNeighbors() ? myOverlayKey : packet->getSource(), routingTable.getSplitSize(), true);

		response->setContactArraySize(ovlyneighbors.size());
		response->setPathArraySize(ovlyneighbors.size());
		response->setSeqNoArraySize(ovlyneighbors.size());
		response->setPathInfoAgeArraySize(ovlyneighbors.size());

		for (uint i = 0; i < ovlyneighbors.size(); i++)
		{
			response->setContact(i, ovlyneighbors[i]->getKey());
			response->setPath(i, ovlyneighbors[i]->getPath());
			response->setSeqNo(i, ovlyneighbors[i]->getStateSeqNo());
			response->setPathInfoAge(i, ovlyneighbors[i]->getAge());
		}
	}

	sendPacket(std::move(response));
}


bool R2Kademlia::handlePNDiscResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadPNDiscRsp>(answer);
	EV_TRACE << "PNDiscRsp from " << response->getSource().shortStr() << " received " << response->getContactArraySize()
		 << " neighbors SSeq#" << response->getStateSeqNo() << endl;

	// update sequence number or restore direct link
	routingTable.addPath(response->getSource(), kad_path(), response->getStateSeqNo(), 0);
	// update PNsyncSeqNo
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(response->getSource())); contact_p != nullptr) {
		contact_p->setSyncPNSeqNo(response->getStateSeqNo());
	}
	else
		EV_ERROR << "PN not found as contact in RT!" << endl;
	// now read PNs from neighbor
	readRTableRsp(*response);
	return true;
}

	
bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	ASSERT(rpcState.original_packet.get() != nullptr);
	if (rpcState.status != RPC_SUCCESS) {
		return true;
	}
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	readRTableRsp(*response);
	return true;
}


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
		auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
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
	auto packet = makePacket<KadFindNodeReq>(*viacontact_p, "FindNode");
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


void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	OverlayKey& query = packet->getDstNodeID(); // get the destination ID
	bool is_rediscovery= packet->getRediscovery();

	EV_TRACE << "KadFindNodeReq (id=" << packet->getId() << ", nonce=" << packet->getNonce() << ") received query for: " << query.shortStr()
		 << " isTracked=" << packet->isTracked() << (packet->getSource() == query ? " (JOIN)" : "") << (is_rediscovery ? " Rediscovery" : "") << endl;

	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	// contstruct notvia list for filtering contacts
	std::list<std::pair<link_t,age_t>> notvialist;
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
				EV_TRACE << "removed " << packet->getSource() << " from found contacts" << endl;
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
			const KadContact& contact = chooseNextHop(contacts, query);
			EV_TRACE << "KadFindNodeReq received query for: " << packet->getDstNodeID().shortStr() << " next overlay hop is " << contact << endl;
			// note: the log_2 will be smaller for a longer common prefix
			ASSERT((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2());
			if((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
				EV_WARN << "no prefix progress made" << endl;
			}
			ASSERT((contact.getKey() ^ query) < (myOverlayKey ^ query));
			packet->appendRoute(contact);
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
					if (routingTable.isInLastBucket(contact.getKey()) and randomTimerDuration[probeTimerId] > SimTime(1000,SIMTIME_MS)) {
						// send a join now
						sendFindNode(myOverlayKey, false);
						// reset timer for periodic joins to ask more frequently now
						resetTimer(probeTimerId, SimTime(100,SIMTIME_MS));
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
	fillRTableRsp(*answer, contacts);
	countReceived(*packet);
	if (is_rediscovery)
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
				ASSERT(contactp->getRDS().rediscoverQtype != RediscoveryState::rediscoverQ_type::undefined);
				simtime_t rediscoveryWaitTime((contactp->getRDS().rediscoverQtype == RediscoveryState::rediscoverQ_type::urgent) ? urgentRediscoveryWaitTime :
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


// this is called periodically for all modules in Statistics::sample()
void R2Kademlia::sampleStatistics (Statistics &stats) {
	// every value here will contribute to the average, min, max,
	// avgDiffQuot and quantiles, a histogram is also created
	stats.value("forwardedPackets", forwardedPackets);
	stats.value("sentPackets", sentPackets);
	stats.value("receivedPackets", receivedPackets);
	stats.value("sentVolume", sentVolume);
	stats.value("receivedVolume", receivedVolume);
	stats.value("forwardedVolume", forwardedVolume);

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
		stats.value("routingTablePathStretch", routingTable.getAveragePathStretch());
	}
	//stats.value("routingTableFoundEquivalentPaths", routingTable.getNumFoundEquivPaths());
	stats.value("fwdTableSize", fwdTablePathID.getNumEntries());
	stats.value("fwdTableValid", fwdTablePathID.getNumValidEntries());
	stats.value("fwdTableForeign", fwdTablePathID.getNumForeignEntries());
	auto [numPrecalcEntries, numUsedPrecalcEntries]= fwdTablePathID.getNumUsedPrecalcEntries();
	stats.value("fwdTablePrecalc", numPrecalcEntries);
	stats.value("fwdTableUsedPrecalc", numUsedPrecalcEntries);
}


// gets called from KadObserver::recordStatistics()
// in finish() during sampling statistics from nodes
// (this is usually a reduced set, not all nodes!)
void R2Kademlia::recordScalars() {
	recordScalar("forwardedPackets", forwardedPackets);
	recordScalar("droppedPackets", droppedPackets);
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
	recordScalar("routingTableContacts", routingTable.getNumContacts());
	recordScalar("routingTableBuckets", routingTable.getNumBuckets());
	recordScalar("routingTablePathLength", routingTable.getAveragePathLength());
	recordScalar("routingTablePathStretch", routingTable.getAveragePathStretch());
	recordScalar("routingTableUpdates", routingTable.getUpdateCount());
	recordScalar("routingTableReplacements", routingTable.getReplacementCount());
	recordScalar("discoveredVicinitySize", vicinity.size());
	nodeid_t nid =  Node::getNode(this)->getNodeId();
	recordScalar("nodeId", nid);
	recordScalar("degree", getNetworkManager()->getTopology().getNodeDegree(nid));
	recordScalar("popularity", getKadObserver().getPopularity(myOverlayKey));
	recordScalar("fwdTableSize", fwdTablePathID.getNumEntries());
	recordScalar("fwdTableValid", fwdTablePathID.getNumValidEntries());
	recordScalar("fwdTableForeign", fwdTablePathID.getNumForeignEntries());
	auto [numPrecalcEntries, numUsedPrecalcEntries]= fwdTablePathID.getNumUsedPrecalcEntries();
	recordScalar("fwdTablePrecalc", numPrecalcEntries);
	recordScalar("fwdTableUsedPrecalc", numUsedPrecalcEntries);

	// EV_TRACE << (shortenedlaterpaths * 100.0) / (double) laterpaths << "% "
	// 	 << shortenedlaterpaths << "/" << laterpaths << " of later paths shortened. "
	// 	 << "Viable PIDs: " << viablePIDpaths << "(" << (viablePIDpaths*100.0)/(double) laterpaths << "%)" << endl;
}

void R2Kademlia::recordRoutingTable() {
	// these submodules are a hack to record scalars for the contacts
	// they are only used for scalar recording
	cModuleType* modType = cModuleType::get("routingsim.routing.r2kademlia.Dummy");
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
NodePkt_p R2Kademlia::makeHelloPacket()
{
	KadHello_p hello = std::make_unique<KadHello>();
	OverlayKey k(0);
	hello->setName("KadHello");
	hello->appendHop(k);
	hello->setSource(myOverlayKey);
	hello->setHopLimit(hopLimit);
	hello->setSourceId(node().getNodeId());
	hello->setNonce(0);
	hello->setStateSeqNo(getLSSeqNo());
	hello->addByteLength(OverlayKey::getLength()+8+32);
	NodePkt_p nodePkt = std::make_unique<NodePkt>(hello->getName());
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
	nodePkt->encapsulate(hello.release());

	return nodePkt;
}


void R2Kademlia::applyShortcut(KadPacket& packet)
{
	auto knownContact= routingTable.getContact(packet.getTarget());

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
		else
			EV_WARN << "Contact in non valid state: " << *knownContact;
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
				packet.setPathInfoAge(i, contacts[i]->getAge());
				packet.addBitLength(contacts[i]->getPath().size()*OverlayKey::getLength()+32+sizeof(age_t)*8);

			}
		}
		packet.addBitLength(contacts.size()*OverlayKey::getLength());
	}
}


void R2Kademlia::readRTableRsp(const KadRTableRsp& packet) {
	ASSERT(packet.getContactArraySize() == packet.getPathArraySize());
	kad_path viaPath = packet.reversePath(packet.getIntermediatePath());
	viaPath.push_back(packet.getSource());
	// should shorten path here already so that the in loop condition fits
	routingTable.shortenPath(viaPath);

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
		if ( not(routingTable.addPath(packet.getContact(i), packet.getPath(i), packet.getSeqNo(i), packet.getPathInfoAge(i), viaPath)) ) {
			// even if contact was not interesting for RT, add it as PID entry if within vicinitydiscovery radius
			// the following condition is correct because the target contact 
			// is not contained in (viaPath + packet.getPath(i))
			bool querynode(false);
			if ( packet.getContact(i)!= myOverlayKey and ((viaPath.size() + packet.getPath(i).size()) < nodeVicinityGraph::vicinityDiscoveryRadius) )
			{
				// for further processing we need a temporary contact object
				// since it was not put into the routingTable
				KadContact neighborhoodContact(packet.getContact(i),
							       routingTable.concatViaPath(viaPath, packet.getPath(i)),
							       packet.getSeqNo(i));

				if (auto vn= vicinity.getNode(neighborhoodContact.getKey()); vn == nullptr) {
					// newly put intro vicinity, so get its PNs
					querynode= true;
				}
				else {
					// newer sequence number for existing neighbor? reget PNs
					if (vn->stateseqno < neighborhoodContact.getStateSeqNo())
						querynode= true;
				}

				// try to add node
				bool nodeOrLinkAdded= vicinity.addNode(neighborhoodContact.getPath().empty() ? myOverlayKey : neighborhoodContact.getPath().back(), // parent node
								  neighborhoodContact.getKey(), // new child node
								  neighborhoodContact.getStateSeqNo(),
								  neighborhoodContact.getPath().size()+1); // distance of child node to us

				// still room for one more hop in neighborhood exploration?
				// the same is done for new contacts in RT (see onNewContact)
				//bool expandDiscovery(viaPath.size() + packet.getPath(i).size() + 1  < nodeVicinityGraph::vicinityDiscoveryRadius);
				// add PID entry as this neighbor is in our vicinityDiscoveryRadius
				if (usePathIDs and nodeOrLinkAdded) {
					fwdTablePathID.addPrecalcEntry(&neighborhoodContact);
					vicinityPathIDCompletion(neighborhoodContact);
				}
			} // endif entry within vicinitydiscovery

			// only get further PNs from node if strictly smaller than discovery radius
			// this should improve the routing table contacts
			if ( querynode and ((viaPath.size() + packet.getPath(i).size() + 1) < discoveryRadius) ) {
				KadContact neighborhoodContact(packet.getContact(i),
							       routingTable.concatViaPath(viaPath, packet.getPath(i)),
							       packet.getSeqNo(i));

				sendGetPhysicalNeighbors(&neighborhoodContact);
			}
		} // endif: new path not added
	} // end for each reported contact
}




// check whether we need to resync with physical neighbor
// normal stateSeqNo not sufficient, because it may have been updated by another message already
// getSyncPNSeqNo() tells us when we received the last full PN list from this PN
bool R2Kademlia::checkForResync(const OverlayKey& contact, stateseqno_t newseqno) {
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(contact)); contact_p != nullptr and contact_p->isPhysicalNeighbor()) {
		contact_p->updateLastSeen();
		// if state seqno of PN has changed resync
		if (contact_p->getSyncPNSeqNo().isInvalid()) {
			// do it immediately
			EV_TRACE << "resync required for " << contact_p->getKey().shortStr() << " doing it now" << endl;
			sendDiscoveryReq(contact_p);
		}
		else
		if (newseqno > contact_p->getSyncPNSeqNo()) {
			EV_TRACE << "resync required for " << contact_p->getKey().shortStr() << " new: "
				 << newseqno << " old: " << contact_p->getSyncPNSeqNo() << endl;

			reSyncPN(*contact_p, newseqno);

			return true;
		}
	}

	return false;
}


/// schedule sending PNDiscReq for contacts, because meanwhile more neighbors will be discovered
/// also record expected new seqno, so if state had been updated meanwhile we can avoid resync
/// this will avoid triggering frequent resynchronizations
void R2Kademlia::schedulePNDiscReq(const OverlayKey& neighbor, stateseqno_t expected_seqno)
{
	if (not PNDiscoverQueue.empty()) {
		if (auto it= std::find_if(PNDiscoverQueue.begin(), PNDiscoverQueue.end(),
					  [&](const auto& p) { return p.first == neighbor; });
		    it != PNDiscoverQueue.end()) {
			// same neighbor already present, but
			// probably need to update the state seq no
			if (expected_seqno.isInvalid()) {
				// update to invalid
				it->second= expected_seqno;
			}
			else {
				// only update when actually larger
				if (expected_seqno > it->second)
					it->second= expected_seqno;
			}
			EV_TRACE << "schedulePNDiscReq(): " << neighbor.shortStr() << " already queued, updated expected SSeq#" << expected_seqno << endl;
			return;
		}
	}

	PNDiscoverQueue.push_back(std::make_pair(neighbor, expected_seqno));
	EV_TRACE << "schedulePNDiscReq(): " << neighbor.shortStr() << " queued, expected  seq#" << expected_seqno << endl;
	if (not pndiscScheduled)
	{
		simtime_t wait_time(SimTime(uniform(PNDiscrecoveryWaitTime/2,
						    3*PNDiscrecoveryWaitTime/2),SIMTIME_US));
		EV_TRACE << "scheduling PNDiscReq for " << neighbor.shortStr()
			 << " in " << wait_time << 's' << endl;
		addOneShotTimer(wait_time, std::bind(&R2Kademlia::sendScheduledPNDiscReq, this));
		pndiscScheduled= true;
	}
}



void R2Kademlia::sendScheduledPNDiscReq() {
	// traverse list of scheduled contact keys for path setup
	// recheck status and set to rediscover
	EV_TRACE << "sendScheduledPNDiscReq() called, Queued: " << PNDiscoverQueue.size() << endl;

	while (!PNDiscoverQueue.empty())
	{
		auto& [neighborcontactkey, expected_seqno]= PNDiscoverQueue.front();
		// check for current status of scheduled contact
		if (neighborTable.isNeighbor(neighborcontactkey)) { // so contact is still an active neighbor
			// note: contact may have been invalidated or changed meanwhile
			if (auto rt_contact= routingTable.getContact(neighborcontactkey); rt_contact != nullptr) { // so contact still exists
				// check whether update is needed (invalidSeqNo means unconditionally update)
				if (expected_seqno.isInvalid() or expected_seqno > rt_contact->getSyncPNSeqNo())
					sendDiscoveryReq(rt_contact);
				else
					EV_TRACE << "Obsolete PNDiscReq for " << rt_contact->getKey().shortStr() << " avoided due to current seqno "
						 << rt_contact->getSyncPNSeqNo() << " expected: "
						 <<  expected_seqno << endl;
			}
			else
				EV_ERROR << "PNDiscReq cannot be sent, because contact " << neighborcontactkey.shortStr() << " not found!?" << endl;
		}
		else
			EV_ERROR << "PNDiscReq cannot be sent, because contact " << neighborcontactkey.shortStr() << " not in neighbor table anymore" << endl;
		PNDiscoverQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	pndiscScheduled= false;
}



void R2Kademlia::countReceived(KadPacket& packet) {
	//EV_TRACE << "countReceived: tracked=" << packet.isTracked() << endl;
	receivedPackets++;
	receivedVolume+= packet.getByteLength();

	// registerPacketDelivered will also count first packet stretch
	getKadObserver().registerPacketDelivered(packet);
	ASSERT(packet.getControlInfo() != nullptr);
	getStatistics().deliveredMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()),
		packet.getProgress()
	);
}


void R2Kademlia::countDropped(KadPacket& packet, const OverlayKey& target, uint8_t reason) {
	droppedPackets++;
	if (packet.isTracked())
		EV_TRACE << "+DROPPED+ nonce:" << packet.getNonce() << " reason:" << (unsigned int) reason << endl;

	getKadObserver().registerPacketDropped(packet, myOverlayKey, target, reason);
	nodeid_t nid = undefined_node;
	if (getKadObserver().exists(target))
		nid = getKadObserver().getNodeId(target);
	ASSERT(packet.getControlInfo() != nullptr);
	getStatistics().dropMessage(
		check_and_cast<cMessage *>(packet.getControlInfo()), nid
	);
}


void R2Kademlia::returnToSender(uint8_t errorType, KadPacket_p packet) {
	KadError_p answer = makeAnswer<KadError>(*packet);
	answer->setErrorType(errorType);
	
	EV_TRACE << "returnToSender() - error: " << (unsigned int) errorType << " - return path:";
	for (const auto& c : answer->getSrcPath())
		EV_TRACE << ' ' << c.shortStr();
	EV_TRACE << endl;		
	answer->encapsulate(packet.release());
	if (answer->hasRemainingHops())
		sendPacket(std::move(answer));
	else // we sent this packet!
		handleAnswer(std::move(answer));
}


template<typename T> bool R2Kademlia::handleIfFitting(KadPacket_p& packet) {
	ASSERT(packet); // no nullptr here!
	KadPacket *raw_ptr = packet.get();
	//EV_TRACE << "handleIfFitting raw_ptr " << static_cast<void*>(raw_ptr) << endl;
	std::unique_ptr<T> tPacket(dynamic_cast<T*>(raw_ptr));
	// if cast failed we get nullptr -> skip the rest, return false
	if (not tPacket) return false;
	// invalidate the original unique_ptr
	(void) packet.release();
	//EV_TRACE << "handleIfFitting tPacket " << static_cast<void*>(tPacket.get()) << endl;
	// delegate responsibility to handlePacket method
	handlePacket(std::move(tPacket));
	return true;
}

} //namespace
