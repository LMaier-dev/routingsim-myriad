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
		par("pathSimplification").boolValue(),
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
	rediscoveryWaitTime = (simtime_t) (par("rediscoveryWaitTime").doubleValue() * 0.001); // given in ms
	updateWaitTime = (simtime_t) (par("updateWaitTime").doubleValue() * 0.001);  // given in ms
	updateWaitTimeFast = updateWaitTime / 10;
	pathSetupWaitTime = (simtime_t) (par("pathSetupWaitTime").doubleValue() * 0.001);  // given in ms

	getStatistics().addSample(this);
	simtime_t probeInterval = (simtime_t) par("probeInterval").doubleValue();
	// doProbeSelf is required for joining (will do exponential backoff)
	// calculation for first timer is: initialOffset - duration/2 + random([0..duration])
	// start after 100ms, use 250ms, 500ms, 1s, 2s, 4s, 8s, 16s, 32s, 64s, 128s, 256s, 300s as intervals
	addTimer(SimTime(225,SIMTIME_MS), SimTime(250,SIMTIME_MS), std::bind(&R2Kademlia::doProbeSelf, this, ph::_1));
	// regularly probe for closest neighbors
	addTimer(SimTime(1000,SIMTIME_MS), probeInterval*2, std::bind(&R2Kademlia::doProbeNeighbors, this, ph::_1));
	// regularly probe for non PN contacts to test path liveliness
	addTimer(SimTime(1000,SIMTIME_MS), probeInterval, std::bind(&R2Kademlia::doProbeContacts, this, ph::_1));
	// regularly send FindNodeReq to randomly chosen IDs
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doProbe, this, ph::_1));
	// check whether other existing nodes can be reached (uses KadObserver)
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doReachabilityTest, this, ph::_1));
	if (sendDataPackets)
	{
	  EV << "will send data packets after " << getSimulation()->getWarmupPeriod() << endl;
	  addTimer(getSimulation()->getWarmupPeriod()+probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doDataProbe, this, ph::_1));
	}

	WATCH(myOverlayKey);
	WATCH(glsseqno);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);
	WATCH_OBJ(fwdTablePathID);

	// the following is not necessary, because onConnect will do this
	//sendToNeighbors(makeHelloPacket());
}


void R2Kademlia::onConnect( nodeid_t nid ) {
	incGLSSeqNo();
	sendToNode(nid, makeHelloPacket(), SimTime(10,SIMTIME_US));
}


void R2Kademlia::onDisconnect( nodeid_t nid ) {
	//EV_TRACE << "onDisconnect " << nid << endl;
	// if this node is taken down, the packets arrive after the reset
	if(neighborTable.size() == 0) return;
	OverlayKey key = neighborTable.getOverlayKey(nid);
	ASSERT(not key.isUnspecified());
	ASSERT(neighborTable.getNodeId(key) == nid );

	incGLSSeqNo();
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
	scheduleRediscovery(key, key, myOverlayKey, routingTable.isInLastBucket(key));

	// contacts from routing table with this neighbor as next hop are not usable until rediscovered
	routingTable.invalidateContactsPN(key);

	// send UpdateRouteReq w/ failed link to ID-wise closest Nodes
	// we can rely on routingTable.getContact(key) != nullptr, because of earlier setContactInvalid() call
	scheduleUpdate(*(routingTable.getContact(key)), KAD_Unreachable);
}


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

	// other packets should do so
	packet->advanceProgress(); // increase hop count
	if (packet->getCurrentHop() != myOverlayKey) {
		EV_ERROR << "dropping packet, current hop does not fit!" << endl;
		//TODO how does this happen?
		countDropped(*packet, packet->getTarget(), wrongNextHop);
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

		bool is_rediscovery(false);
		//TODO ERROR OVERHEARING, more overhearing
		// special actions while forwarding packets
		switch (packet->getType())
		{
			case KAD_FindNodeReq:
			        {
					// check for rediscovery packet
					const KadFindNodeReq* findnodereq= dynamic_cast<const KadFindNodeReq*>(packet.get());
					is_rediscovery= findnodereq->getRediscovery();
			        }
				break;

			case KAD_PathSetupReq: // allowed to take shortcuts
				// TODO send back response if path ID already installed and still valid
				applyShortcut(*packet);
				forwardedPathSetup++;
				break;

			case KAD_PathSetupRsp:
				EV_TRACE << "forwarding PathSetupRsp, origin source path: <"
					 << packet->reversePath(packet->getSrcPath()) << endl;
				// add potential pathID for destination at this intermediate node
				checkForPathInstallation(*packet);
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
				routingTable.addPath(packet->getSource(), path_of_interest, packet->getStateSeqNo(), 0, KadContact::PathProperty::indirect);
			}

			// see comment above: rediscovery messages cannot be used for improving routing information
			if (not is_rediscovery)
			{
				path_of_interest= packet->getSrcPath(packet->getProgress()+1, packet->getRouteArraySize()-1);
				if (path_of_interest.size() > 0)
				{
					routingTable.addPath(packet->getTarget(), path_of_interest, packet->getTargetSeqNo(), 0, KadContact::PathProperty::indirect);
				}
			}
		}

		forwardPacket(std::move(packet), std::move(pkt));
		return;
	}
	else { // no hops left, packet is addressed to me
		packet->setControlInfo(pkt.release());
		
		// learn route from packet
		//TODO optimization use isPathInteresting

		// Paths learned from incoming requests are forced by source routing and thus not directly learned
		KadContact::PathProperty pathproperty= KadContact::PathProperty::undefined;
		switch (packet->getType()) {
			case KAD_FindNodeRsp:
			case KAD_QueryRouteRsp:
			case KAD_ProbeRsp:
			case KAD_PathSetupRsp:
			{
				const KadAnswer* answer= dynamic_cast<const KadAnswer*>(packet.get());
				ASSERT(answer);
				pathproperty= answer->getShortened() ? KadContact::PathProperty::indirect : KadContact::PathProperty::direct;
				break;
			}
			default:
				pathproperty= KadContact::PathProperty::indirect;
				break;

		};

		// evaluate return path for routing table
		routingTable.addPath(packet->getSource(), packet->reversePath(packet->getIntermediatePath()), packet->getStateSeqNo(), 0, pathproperty);

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



void R2Kademlia::onSelfMessage(cMessage_p msg) {
	std::unique_ptr<RPCtimeoutMsg> rpctimeout(dynamic_cast<RPCtimeoutMsg *>(msg.get()));
	if (rpctimeout)
	{
		msg.release(); 		// no double deletion

		//EV_TRACE << "RPC Timeout Message for nonce " << rpctimeout->nonce << endl;

		handleTimeOut(rpctimeout->nonce);

		// timeout message will be dismissed
		return;
	}

	// handle timer
	// timerId == -1 is a one shot timer
	short timerId = msg->getKind();

	if (cbfunc_t* timerCallback(static_cast<cbfunc_t *>(msg->getContextPointer()));
	    timerCallback != nullptr) {
		// now call the callback function
		(*timerCallback)(timerId);

		// delete the callback function for one shot messages
		if (timerId == -1) {
			delete timerCallback;
			msg->setContextPointer(nullptr);
		}
	}

	// reschedule timer message if duration > 0, otherwise it was a one shot timer
	if (timerId  >= 0)
	{
		ASSERT((size_t)timerId < timerDuration.size());
		ASSERT(timerMsg[timerId] == msg.get());
		if (timerDuration[timerId] > 0)
		{
			simtime_t time = uniform(timerDuration[timerId]/2, timerDuration[timerId]/2*3);
			scheduleAt(simTime() + time, msg.release());
		}
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


void R2Kademlia::handlePacket(KadHello_p packet) {
	EV << "received hello packet from " << packet->getSource().shortStr() << " StateSeq#" << packet->getStateSeqNo() << endl;

	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	//EV_TRACE << "neighbor table: "<< neighborTable << endl;

	// PNDiscovery should be sent before any further messages to avoid missing neighbors
	// send back PNDiscReq to get contacts from physical neighbor
	//TODO wait random time
	KadContact neighborcontact(packet->getSource(), kad_path());
	sendDiscoveryReq(&neighborcontact);

	// try to add neighbor to routing table
	if (routingTable.addPath(packet->getSource(), kad_path(), packet->getStateSeqNo(), 0, KadContact::PathProperty::direct))
	{
		EV_TRACE << "adding neighbor successful, seq#" << packet->getStateSeqNo() << endl;
	}
	else
		EV_TRACE << "adding neighbor failed (maybe was already there)" << endl;

	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		contact_p->updateLastSeen();
	}

	//EV_TRACE << "handlePacket: "<< static_cast<void*>(packet.get()) << endl;
}


void R2Kademlia::handlePacket(KadAnswer_p packet) {
	//EV_DEBUG << "received answer packet (for nonce=" << packet->getNonce() << ") from " << packet->getSource().shortStr() << endl;
	if (packet->getType() == KAD_Error) {
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
		// note that handleAnswer() will be called  afterwards for processing the RPC result
	}
	else if (packet->getType() == KAD_FindNodeRsp) {

		// update StateSeqNo and lastseen of origin contact
		if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
			contact_p->setStateSeqNo(packet->getStateSeqNo());
			contact_p->updateLastSeen();
		}

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
		getKadObserver().sampleLaterPath(getOverlayKey(),returnpath);

	        // uint pathlen= returnpath.size();
		// // start from farthest node to find potential PID entry
		// for (auto keyit= returnpath.crbegin(); keyit != returnpath.crend(); keyit++) {
		// 	if ( fwdTablePathID.NIDPIDTable.find(*keyit) != fwdTablePathID.NIDPIDTable.end() )
		// 	{
		// 		EV_TRACE << "LaterPath: found a PID for " << pathlen << "/" << returnpath.size() << endl;
		// 		// the end of this path is only useful
		// 		if (routingTable.distance(*keyit,returnpath.back())<routingTable.distance(returnpath.back()))
		// 		{
		// 			viablePIDpaths++;
		// 			break;
		// 		}
		// 	}
		// 	pathlen--;
		// } // end for

	}

	// the following call handles the RPC answer
	handleAnswer(std::move(packet)); // see RPC.cc
}


void R2Kademlia::forwardPacket(KadPacket_p packet, NodePkt_p nodePkt) {
	// we do not route hello packets!
	ASSERT(packet->getType() != KAD_PNHello);
	ASSERT(packet->hasRemainingHops());
	ASSERT(packet->getControlInfo() == nullptr);

	// TODO Check for already present NotVia() => loop
	// TODO Check for presence of own ID in so far processed path (maybe as overlay hop)
	
	//TODO optimization: potential shortcut - we check if the target is a already neighbor
	// only allowed for exact and not strict routing

	// it is not allowed to use a reference for nextHop here,
	// because modRoute() may delete the referenced object and nextHop is used thereafter
	const OverlayKey nextHop(packet->getNextHop());
	nodeid_t nid(neighborTable.getNodeId(nextHop));

	EV_TRACE << "forwardPacket(): " << packet->getName() << " dest " << packet->getTarget().shortStr()
		 << " next hop: " << nextHop.shortStr() << " (" << nid << ')'
		 << " nonce=" << packet->getNonce() << endl;

	if (nid != undefined_node) {
		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt));

		forwardedPackets++;

		return;
	}
	else { // next hop not available
		// for Routing Updates and Rediscovery messages try to use overlay-based forwarding
		EV_TRACE << "forwardPacket(): next hop unavailable, path <";
		for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
			EV_TRACE << ' ' << packet->getRoute(i).shortStr();
		EV_TRACE << " > progress=" << packet->getProgress() << endl;

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
			// due to broken link < b20388 d65e04 > substituted by <f8d1e5 2550d5 d65e04>
			routingTable.simplifyPath(newpath);
			EV_TRACE << "forwardPacket(): reroute PN, contact " << *contact << endl;
			// rerouting makes this intermediate node to overlay hop
			packet->setActive(packet->getProgress(), true);
			packet->modRoute(newpath);
			// note: targetSeqNo does not need to be adapted as the final hop is not modified

			EV_TRACE << "forwardPacket(): reroute path <";
			for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
				EV_TRACE << ' ' << packet->getRoute(i).shortStr();
			EV_TRACE << " > progress=" << packet->getProgress() << endl;

			if (detour) {
				// new next hop
				nid = neighborTable.getNodeId(newpath.front());
				if (nid != undefined_node) {
					// TODO check whether this actually makes sense for all packet types?
					packet->addNotViaLink({nextHop,myOverlayKey}, true);
					EV_TRACE << "forwardPacket(): forwarding to next hop (" << nid << ")" << endl;

					nodePkt->encapsulate(packet.release());
					sendToNode(nid, std::move(nodePkt));
					forwardedPackets++;
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
				packet->addNotViaLink({nextHop,myOverlayKey}, true);
			}

			// try to reroute
			reroutePacket(*finaldestination, std::move(packet), std::move(nodePkt));
			return;
		}
	} // end else (no next hop found)

	EV_ERROR << "forwardPacket(): could not find next hop!" << endl;
	onMissingNeighbor(std::move(packet), std::move(nodePkt));
}


void R2Kademlia::reroutePacket(const OverlayKey& finaldest, KadPacket_p packet, NodePkt_p nodePkt)
{
	std::list<link_t> notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(packet->getFailedLink(i));
		}
	}

	KadContactList ovlyneighbors= routingTable.getClosestContacts(finaldest, routingTable.getSplitSize(), true, notvialist);
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
			packet->setTargetSeqNo(contact.getStateSeqNo());
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
				nodePkt->encapsulate(packet.release());
				sendToNode(nid, std::move(nodePkt));
				forwardedPackets++;
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
			EV_TRACE << "reroutePacket(): cannot find alternative route via overlay" << endl;
			break;
		}
	} // end while

	EV_ERROR << "reroutePacket(): no contacts found" << endl;

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


void R2Kademlia::sendPacket(KadPacket_p packet) {
	ASSERT(packet->getSource() == myOverlayKey);
	ASSERT(packet->getControlInfo() == nullptr);
	auto nodePkt = std::make_unique<NodePkt>(packet->getName());
	nodePkt->setType(KAD_CTRL_PACKET_TYPE);
	if (packet->getType() == KAD_FindNodeReq and check_and_cast<KadFindNodeReq*>(packet.get())->getExact()) {
		getKadObserver().registerPacket(*packet);
		getStatistics().registerMessage(nodePkt.get());
	}
	// EV_TRACE << "sendPacket(): " << packet->getName() << ", " << (int) packet->getType() << endl;
	sentPackets++;
	forwardPacket(std::move(packet), std::move(nodePkt));
}


/**
 * Assumption is that the contact list is sorted by XOR and distance to target  */
const KadContact& R2Kademlia::chooseNextHop(const KadContactList& contacts, const OverlayKey& target) {
	if (proximityRouting and contacts.size() > 1) {
		// dprefix is the number of non-matching bits with target: smaller dprefix is closer to target
		int dprefix = target.getLength()+1; // keylength + 1
		size_t dproxMin = hopLimit;

		// look for first usable contact
		const KadContact* contact(nullptr);
		auto it = contacts.begin();
		for (; it != contacts.end(); it++)
		{
			contact= *it;
			if (contact != nullptr)
			{
				dprefix = (contact->getKey() ^ target).log_2();
				dproxMin = contact->getPathLength();
				if ((myOverlayKey ^ target).log_2() == dprefix) {
					// if we have no prefix progress,
					// we take the closest contact by XOR, which is this one (assumption of XOR-sorted contacts)
					return *contact;
				}
				break;
			}
		}
		// dprefix, dproxMin now initialized to first non-null contact (if any)
		// continue with next contact
		if (it != contacts.end())
			it++;
		// look for closest contact with shortest path
	        for (; it != contacts.end(); it++) {
			if ( (*it) != nullptr ) {
				// stop at contact that is further away
				if (((*it)->getKey() ^ target).log_2() > dprefix)
					break;
				// use contact with shortest path
				if ((*it)->getPathLength() < dproxMin) {
					dproxMin = (*it)->getPathLength();
					contact = *it;
				}
			}
		} // end for

		if (contact != nullptr)
			return *contact;
	}
	else {
		// no proximity routing or only single contact
		//ASSERT(contacts.front()->getKey() != target); // this assertion seems to be wrong

		if (not contacts.empty() and contacts.front() != nullptr) {
			if (contacts.front()->getKey() == target)
			{
				EV_WARN << "Next hop is the target, number of contacts: " << contacts.size()
					<< "contact: " << *(contacts.front()) << endl;
			}
			return *(contacts.front());
		}
	}

	// if everything else fails we return an empty contact
	return KadContact::undefinedContact;
}

// -------------------- RPC methods ---------------------------


// PING
void R2Kademlia::sendProbe(KadContact_p contact) {
	EV_TRACE << "sending ProbeReq from " << myOverlayKey.shortStr() << " to " << contact->getKey().shortStr() << endl;
	sendRpcCall(
		makePacket<KadProbeReq>(contact, "KadProbeReq"),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleProbeResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2));
}


void R2Kademlia::handlePacket(std::unique_ptr<KadProbeReq> packet) {
	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		contact_p->setStateSeqNo(packet->getStateSeqNo());
		contact_p->updateLastSeen();
	}

	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}


bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(probeRsp->getSource())); contact_p != nullptr) {
		contact_p->setStateSeqNo(probeRsp->getStateSeqNo());
		contact_p->updateLastSeen();
	}

	return true;
}




// Request Physical Neighbors from contact
void R2Kademlia::sendGetPhysicalNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request (PN) to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	req->setOverlayNeighbors(false);
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
	);
	queriedNeighbors.insert(contact->getKey());
}


// Request k nearest Overlay Neighbors from contact 
void R2Kademlia::sendGetOverlayNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request (ON) to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(0); // no physical neighbors
	req->setOverlayNeighborsSource(true); // get known overlay neighbors for my ID
	req->setContactsOnly(false); // get also the routes
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleQueryRouteResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
	);
}


// Send PNDiscReq
void R2Kademlia::sendDiscoveryReq(KadContact_p contact)
{
	ASSERT(contact != nullptr);
	EV_TRACE << "sending PNDisc request to " << *contact << endl;
	auto req = makePacket<KadPNDiscReq>(contact, "KadPNDiscReq");
	req->setSourceId(node().getNodeId());
	req->setNeighborhood(1); // get only physical neighbors (1 hop away)
	req->setContactsOnly(false);
	//req->setHopLimit(1);
	sendRpcCall(
		std::move(req),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handlePNDiscResponse, this, ph::_1, ph::_2),
		std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
	);
	queriedNeighbors.insert(contact->getKey());
}


// handle PNDiscReq
void R2Kademlia::handlePacket(std::unique_ptr<KadPNDiscReq> packet) {
	EV_TRACE << "received PNDiscReq from " << packet->getSource().shortStr() << endl;

	// since this is physical neighbor add it to our neighbortable
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	EV_TRACE << "neighbor table: "<< neighborTable << endl;

	// add physical contact to routing table or update sequence number
	routingTable.addPath(packet->getSource(), kad_path(), packet->getStateSeqNo(), 0, KadContact::PathProperty::direct);

	// update lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		contact_p->updateLastSeen();
	}

	// TODO currently only getting physical neighbors is implemented
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadPNDiscRsp>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	response->setContactArraySize(neighbors.size());
	response->setPathArraySize(neighbors.size());
	response->setSeqNoArraySize(neighbors.size());
	response->setContactAgeArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++) {
		response->setContact(i, neighbors[i]);
		response->setSeqNo(i, routingTable.getContact(neighbors[i])->getStateSeqNo());
		response->setContactAge(i,routingTable.getContact(neighbors[i])->getAge());
	}
	sendPacket(std::move(response));
}



void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet) {
	EV_TRACE << "received KadQueryRouteReq from " << packet->getSource().shortStr() << endl;

	// update StateSeqNo and lastseen of origin contact
	if (KadContact* contact_p= const_cast<KadContact*>(routingTable.getContact(packet->getSource())); contact_p != nullptr) {
		contact_p->setStateSeqNo(packet->getStateSeqNo());
		contact_p->updateLastSeen();
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
		response->setContactAgeArraySize(neighbors.size());
		for (uint i = 0; i < neighbors.size(); i++) {
			if (routingTable.getContact(neighbors[i]) != nullptr)
			{
				response->setContact(i, neighbors[i]);
				response->setSeqNo(i, routingTable.getContact(neighbors[i])->getStateSeqNo());
				response->setContactAge(i,routingTable.getContact(neighbors[i])->getAge());
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
		response->setContactAgeArraySize(allcontacts.size());
		for (uint i = 0; i < allcontacts.size(); i++)
		{
			response->setContact(i, allcontacts[i]->getKey());
			response->setPath(i, allcontacts[i]->getPath());
			response->setSeqNo(i, allcontacts[i]->getStateSeqNo());
			response->setContactAge(i, allcontacts[i]->getAge());
		}
	}

	if (packet->getOverlayNeighbors() or packet->getOverlayNeighborsSource())
	{
		KadContactList ovlyneighbors= routingTable.getClosestContacts(packet->getOverlayNeighbors() ? myOverlayKey : packet->getSource(), routingTable.getSplitSize(), true);

		response->setContactArraySize(ovlyneighbors.size());
		response->setPathArraySize(ovlyneighbors.size());
		response->setSeqNoArraySize(ovlyneighbors.size());
		response->setContactAgeArraySize(ovlyneighbors.size());

		for (uint i = 0; i < ovlyneighbors.size(); i++)
		{
			response->setContact(i, ovlyneighbors[i]->getKey());
			response->setPath(i, ovlyneighbors[i]->getPath());
			response->setSeqNo(i, ovlyneighbors[i]->getStateSeqNo());
			response->setContactAge(i, ovlyneighbors[i]->getAge());
		}
	}

	sendPacket(std::move(response));
}


bool R2Kademlia::handlePNDiscResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadPNDiscRsp>(answer);
	EV_TRACE << "PNDiscRsp received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr()
		 << " SeqNo#" << response->getStateSeqNo() << endl;
	// update sequence number or restore direct link
	routingTable.addPath(response->getSource(), kad_path(), response->getStateSeqNo(), 0, KadContact::PathProperty::direct);

	readRTableRsp(*response);
	return true;
}

	
bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	readRTableRsp(*response);
	return true;
}


// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key, bool exact) {
	EV_TRACE << "sendFindNode for " << key.shortStr() << (exact ? " (NID)" : (key == myOverlayKey ? " (JOIN)" : " (RID)")) << endl; // Node ID or Resource ID?

	//ASSERT(contacts.size() > 0);
	if (auto contacts = routingTable.getClosestContacts(key, findNodeNumber, true); contacts.size() > 0)
	{
		// use several contacts to send multiple Rediscovery requests in parallel
		const KadContact& contact = chooseNextHop(contacts, key);
		ASSERT(contact.isValid());
		ASSERT(!contact.getKey().isUnspecified());
		if (contact.getKey() == myOverlayKey)
		{
			EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
			return;
		}
		auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
		packet->setDstNodeID(key);
		packet->setExact(exact);
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



void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	OverlayKey& query = packet->getDstNodeID(); // get the destination ID
	bool is_rediscovery= packet->getRediscovery();

	EV_TRACE << "KadFindNodeReq (id=" << packet->getId() << ", nonce=" << packet->getNonce() << ") received query for: " << query.shortStr()
		 << " isTracked=" << packet->isTracked() << (packet->getSource() == query ? " (JOIN)" : "") << (is_rediscovery ? " Rediscovery" : "") << endl;

	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	// contstruct notvia list for filtering contacts
	std::list<link_t> notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(packet->getFailedLink(i));
		}
	}
	// ContactList is sorted according to XOR metric with respect to key
	KadContactList contacts = routingTable.getClosestContacts(query, contactsLen, true, notvialist);

	if (query != myOverlayKey) { // someone else routing via us

		// the first contact is closest to query key
		// if it is the source, we would route it back in a loop
		// (may be also the case if a node tries to join and looks up its own node ID)
		if (contacts.empty() == false and contacts.front()->getKey() == packet->getSource()) {
			if (contacts.size() == 1) {
				// the only contact we know is the source
				EV_ERROR << "only one contact known: the source" << endl;
				countDropped(*packet, query, routeFailureDeadEnd);
				returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

				if (is_rediscovery)
					droppedRediscovery++;

				return;
			}
			else // remove source from answer
			{
				contacts.erase(contacts.begin());
			}
		}

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
		// 	EV_TRACE << thiscontact->getKey().shortStr() << " ";
		// }
		// EV_TRACE << endl;

		// precondition here: !contacts.empty(), i.e., we have a contact as potential next hop
		// if next contact is closer to destination than us, it's ok
		if ((contacts.front()->getKey() ^ query) < (myOverlayKey ^ query)
				and contacts.front()->getPathLength() < packet->getHopLimit()) {
			// we have found a contact that is closer than us to target (query)
			// => forward request
			const KadContact& contact = chooseNextHop(contacts, query);
			EV_TRACE << "KadFindNodeReq received query for: " << packet->getDstNodeID().shortStr() << " next overlay hop is " << contact << endl;
			// note: the log_2 will be smaller for a longer common prefix
			ASSERT((contact.getKey() ^ query).log_2() <= (myOverlayKey ^ query).log_2());
			if((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
				EV_WARN << "no prefix progress made" << endl;
			}
			ASSERT((contact.getKey() ^ query) < (myOverlayKey ^ query));
			packet->appendRoute(contact);
			packet->setTargetSeqNo(contact.getStateSeqNo());
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
				// since we need an exact match and cannot make any progress, we are lost here
				EV_ERROR << "cannot reach final destination " << query.shortStr() << " (dead end)" << endl;
				countDropped(*packet, query, routeFailureDeadEnd);
				//TODO: if we find a closer contact too far away, we should send HOP LIMIT EXCEEDED
				returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

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
	fillRTableRsp(*answer, contacts);
	countReceived(*packet);
	if (is_rediscovery)
		succeededRediscovery++;
	
	EV_TRACE << "KadFindNodeReq query for " << query.shortStr() << " answering with list size " << contacts.size() << endl;
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
	// if this was a rediscovery packet, check for its success
	if (findnodereq->getRediscovery() == true) {
		if (auto contactp= routingTable.getContact(findnodereq->getDstNodeID()); contactp != nullptr) {
			if (contactp->isRediscovering()) { // not successful yet
				EV_TRACE << "Rediscovery not successful for contact " << *contactp << ", next try." << endl;
				// send a new find node req
				sendRediscovery(const_cast<KadContact*>(contactp), 1);
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

// ------------------------- policy foo --------------------------------------

void R2Kademlia::onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt) {
//  	getStatistics().dropMessage(pkt, undefined_node);
	EV_WARN << "onMissingNeighbor called - packet next hop should be " << packet->getNextHop().shortStr() << endl;
	droppedPackets++;
	// do not generate Error packets for Error or UpdateRoute packets
	switch (packet->getType()) {
		case KAD_Error:
		case KAD_UpdateRouteReq:
		case KAD_QueryRouteRsp:
		case KAD_FindNodeRsp:	
			EV << "not generating error packet for packet type " << packet->getName() << "(" << (unsigned int) packet->getType() << ")" << endl;
			break;
		default:
		{
			if (packet->getSource() == myOverlayKey)
			{
				EV_DEBUG << "I'm the source, so not returning any message" << endl;
				return;
			}
			else
				returnToSender(KAD_SegmentFailure, std::move(packet));
		}
	} // end switch
}




void R2Kademlia::onErrorPacket(KadError& error) {

	EV_INFO << "received ERROR packet (" << (unsigned int) error.getErrorType() << ") ";
	switch (error.getErrorType()) {
		case KAD_NoError:
			EV_INFO << "No Error";
			break;
		case KAD_HopLimitExceeded:
			EV_INFO << "HopLimitExceeded";
			break;
		case KAD_SegmentFailure:
			EV_INFO << "SegmentFailure";
			break;
		case KAD_NodeUnreachable:
			EV_INFO << "NodeUnreachable";
			break;
		case KAD_PathIDUnknown:
			EV_INFO << "PathIDUnkown";
			break;
		case KAD_RouteFailureDeadEnd:
			EV_INFO << "RouteFailureDeadEnd";
			break;
	}
	EV_INFO	<< endl;
	
	ASSERT(error.getEncapsulatedPacket() != nullptr);
	KadPacket* original = check_and_cast<KadPacket *>(error.getEncapsulatedPacket());
	ASSERT(original->getNonce() == error.getNonce());
	ASSERT(original->getSource() == myOverlayKey);
	switch (error.getErrorType()) {

		// the following errors indicate a problem with the route to the contact
		case KAD_SegmentFailure:
		case KAD_NodeUnreachable:
		{ // Invalidate contact as the error information is current

			// find out the failed link
			auto reportingNode = error.getSource();
			const auto originalPath= original->getSrcPath();
			// find reporting node in the source path
			auto it= find(originalPath.begin(), originalPath.end(), reportingNode);
			// the following should be true, otherwise we are getting
			// an error from some off path node
			ASSERT(it != originalPath.end());
			// the next hop is the one that failed
			auto failedNextHop = (++it != originalPath.end()) ? *it : reportingNode ;

			if (reportingNode != failedNextHop) {
				link_t failedlink(reportingNode, failedNextHop);
				EV_INFO << "Check for invalid contacts because of failed link " << failedlink << " in error packet" << endl;
				// this segment or node error is current and should invalidate all affected contacts
				// a rediscovery will be triggered automatically if necessary
				routingTable.invalidateContacts(failedlink);
			}
			else
				EV_ERROR << "onErrorPacket: failed link collapsed to node " << reportingNode << " path:" << originalPath << endl;
		};
		break;

		case KAD_PathIDUnknown:
		{ // KAD_PathIDUnknown: we should try to resetup the path
			auto affectedContact= routingTable.getContact(original->getTarget());
			if (affectedContact and affectedContact->isValid())
			{
				schedulePathSetup(affectedContact->getKey());
			}
		};
		break;

		default:
			// currently there are no more error types to handle
			break;
	} // end switch
}


// this is called periodically for all sampled modules in Statistics::sample()
void R2Kademlia::sampleStatistics (Statistics &stats) {
	stats.value("forwardedPackets", forwardedPackets-sentPackets);
	stats.value("sentPackets", sentPackets);
	stats.value("receivedPackets", receivedPackets);
	stats.value("forwardedDataPackets", forwardedDataPackets-sentDataPackets);
	stats.value("sentDataPackets", sentDataPackets);
	stats.value("receivedDataPackets", receivedDataPackets);
	stats.value("sentRouteUpdates", sentRouteUpdates);
	stats.value("receivedRouteUpdates", receivedRouteUpdates);
	stats.value("sentPathSetup", sentPathSetup);
	stats.value("receivedPathSetup", receivedPathSetup);
	stats.value("forwardedPathSetup", forwardedPathSetup);
	stats.value("routingTableContacts", routingTable.getNumContacts());
	stats.value("routingTableBuckets", routingTable.getNumBuckets());
	stats.value("routingTableUpdates", routingTable.getUpdateCount());
	stats.value("routingTableReplacements", routingTable.getReplacementCount());
	if(detailedRoutingTableStats) {
		stats.value("routingTablePathLength", routingTable.getAveragePathLength());
		stats.value("routingTablePathStretch", routingTable.getAveragePathStretch());
	}
	stats.value("routingTableIndirectPaths", routingTable.getNumIndirectPaths());
	stats.value("fwdTableSize", fwdTablePathID.getNumEntries());
	stats.value("fwdTableValid", fwdTablePathID.getNumValidEntries());
	stats.value("fwdTableForeign", fwdTablePathID.getNumForeignEntries());
}
// -------------- kad policy -------------

void R2Kademlia::onBucketEmpty(KadBucket& bucket) {
	EV << "bucket empty" << endl;
}


// this is called before deletion of the contact
void R2Kademlia::onContactRemoval(KadContact_p contact)
{
	//EV_TRACE << "onContactRemoval(" << ((contact != nullptr) ? contact->getKey().shortStr() : "nullptr") << ") called" << endl;
	if (contact == nullptr)
		return;

	// delete from Forwarding table PID
	// TODO: this can be scheduled for happening later
	if (usePathIDs)
		fwdTablePathID.delEntry(contact);
}


void R2Kademlia::onNewContact(KadContact_p contact) {

	EV_TRACE << "onNewContact(" << contact->getKey().shortStr() << ") called, SeqNo #" << contact->getStateSeqNo() << endl;
	//ASSERT( getKadObserver().checkSeqNo(contact->getKey(),contact->getStateSeqNo()) == true );

	// physical neighbor, may have been heard of its existence by some third node!
	// avoid further actions until we hear a HELLO
	if (contact->getPathLength() == 0) 
	{ 	if (not neighborTable.isNeighbor(contact->getKey()))
		{
			EV_DEBUG << "onNewContact: will ignore supposed physical neighbor not being in Neighbor table yet" << endl;
			return;
		}
	}

	// ask for physical neighbors if new contact is within discovery radius
	// contact->getPathLength() excludes the destination, so path length = 0 
	// holds for PhyNeighbors (their Physical Neighbors are already discovered by PNDiscReq/Rsp)
	// getPathLength()+1 corresponds to the number of hops from me to the contact.
	// Thus, contact->getPathLength()+1 < discoveryRadius is correct, because we ask
	// those contacts about their PNs which is even one hop further away
	if (contact->getPathLength() > 0 and contact->getPathLength()+1 < discoveryRadius)
	{
		// sends a QueryRouteReq requesting their 1 hop neighborhood
		sendGetPhysicalNeighbors(contact);
	}

	if (contact->isValid())
	{
		// request info about overlay neighbors near my node ID from nodes in our deepest bucket
		if (routingTable.isInLastBucket(contact->getKey()))
		{
			//EV_TRACE << "New contact is in deepest bucket, will ask for contacts close to me" << endl;
			sendGetOverlayNeighbors(contact);
		}

		if (usePathIDs)
			fwdTablePathID.addEntry(contact, false);
	}

	if (usePathIDs and contact->isValid() and contact->getPathProperty() == KadContact::PathProperty::indirect
	    and contact->getPathLength() > discoveryRadius)
	{ // send a path setup request packet for installing PIDs in intermediate nodes for this indirectly learned route
		// only do this for paths longer than two hops
		EV_TRACE << "Triggering path setup to " << contact->getKey().shortStr() << endl;
		schedulePathSetup(contact->getKey());
	}

	scheduleUpdate(*contact, KAD_Announce);
}


void R2Kademlia::onImprovement(const KadContact& contact) {
	// path got improved for this contact -> potentially send UpdateRouteReq
	EV_TRACE << "onImprovement(" << contact.getKey().shortStr() << ") called, SeqNo #" << contact.getStateSeqNo() << ' ' << contact.getPath() << endl;

	//ASSERT( getKadObserver().checkSeqNo(contact.getKey(),contact.getStateSeqNo()) == true );
	if (contact.isValid())
	{
		if (usePathIDs) fwdTablePathID.modEntry(&contact, false);
	}

	if (usePathIDs and contact.isValid() and contact.getPathProperty() == KadContact::PathProperty::indirect)
	{ // send a path setup request packet for installing PIDs in intermediate nodes for this indirectly learned route
		//EV_TRACE << "Triggering path setup to " << contact.getKey().shortStr() << endl;
		schedulePathSetup(contact.getKey());
	}

	// TODO: wait grace period
	if ( neighborTable.isNeighbor(contact.getKey()) )
	{ // this is true after re-enabling a physical neighbor
		if (not contact.isPhysicalNeighbor())
			EV_WARN << "reenabling physical contact is not flagged as being neighbor" << endl;

		scheduleUpdate(contact, KAD_Announce);
	}
	else
		scheduleUpdate(contact, KAD_Change);
}


void R2Kademlia::reSyncPN(const KadContact& contact) {
	// simply trigger a PNDiscReq to update any sequence number
	EV_TRACE << "reSyncPN triggered for " << contact.getKey().shortStr() << endl;
	sendDiscoveryReq(&contact);
}


void R2Kademlia::onContactCheck(const KadContact& contact) {
	EV_TRACE << "contact check triggered for " << contact.getKey().shortStr() << endl;
	sendProbe(&contact);
}


void R2Kademlia::recordScalars() {
	recordScalar("forwardedPackets", forwardedPackets);
	recordScalar("droppedPackets", droppedPackets);
	recordScalar("sentPackets", sentPackets);
	recordScalar("receivedPackets", receivedPackets);
	recordScalar("forwardedDataPackets", forwardedDataPackets);
	recordScalar("droppedDataPackets", droppedDataPackets);
	recordScalar("sentDataPackets", sentDataPackets);
	recordScalar("receivedDataPackets", receivedDataPackets);
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
	recordScalar("routingTableIndirectPaths", routingTable.getNumIndirectPaths());
	recordScalar("discoveredNeighborhood", queriedNeighbors.size());
	nodeid_t nid =  Node::getNode(this)->getNodeId();
	recordScalar("nodeId", nid);
	recordScalar("degree", getNetworkManager()->getTopology().getNodeDegree(nid));
	recordScalar("popularity", getKadObserver().getPopularity(myOverlayKey));
	recordScalar("fwdTableSize", fwdTablePathID.getNumEntries());
	recordScalar("fwdTableValid", fwdTablePathID.getNumValidEntries());
	recordScalar("fwdTableForeign", fwdTablePathID.getNumForeignEntries());

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
	for (size_type i; i < allContacts.size(); i++) {
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
	hello->setStateSeqNo(getGLSSeqNo());
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
				EV_TRACE << "modifying route due to shortcut: old=" << packet.getSrcPath();

				kad_path newpath(knownContact->getPath());
				newpath.push_back(knownContact->getKey());
				packet.modRoute(newpath);
				packet.setTargetSeqNo(knownContact->getStateSeqNo());


				EV_TRACE << " new=" << packet.getSrcPath() << endl;
			}
		}
		else
			EV_WARN << "Contact in non valid state: " << *knownContact;
	}
}


void R2Kademlia::checkForPathInstallation(KadPacket& packet)
{
	if ( usePathIDs == false )
		return;
	// will install path in data forward direction
	// use existing path from answer (must reverse path)
	KadContact tmpcontact(packet.getSource(),
			      packet.reversePath(packet.getSrcPath(1, packet.getProgress())));

	fwdTablePathID.addEntry(&tmpcontact, true);
}


void R2Kademlia::fillRTableRsp(KadRTableRsp& packet, const KadContactList& contacts, bool includePaths){
	ASSERT(packet.getContactArraySize() == 0);
	ASSERT(packet.getPathArraySize() == 0);
	// in rare cases contact list may be empty
	if (contacts.size() > 0)
	{
		packet.setContactArraySize(contacts.size());
		if (includePaths) {
			packet.setPathArraySize(contacts.size());
			packet.setSeqNoArraySize(contacts.size());
			packet.setContactAgeArraySize(contacts.size());
		}
		for (size_t i = 0; i < contacts.size(); i++) {
			packet.setContact(i, contacts[i]->getKey());
			if (includePaths) {
				packet.setPath(i, contacts[i]->getPath());
				packet.setSeqNo(i, contacts[i]->getStateSeqNo());
				packet.setContactAge(i, contacts[i]->getAge());
			}
		}
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

		if ( not(routingTable.addPath(packet.getContact(i), std::move(packet.getPath(i)), packet.getSeqNo(i), packet.getContactAge(i), viaPath)) ) {
			// even if contact was not interesting for RT, add it as PID entry if within discovery radius
			// the following condition is correct because the target contact 
			// is not contained in (viaPath + packet.getPath(i))
			if ( packet.getContact(i)!= myOverlayKey and ((viaPath.size() + packet.getPath(i).size()) < discoveryRadius) )
			{
				KadContact neighborhoodContact(packet.getContact(i),
							       routingTable.concatViaPath(viaPath, packet.getPath(i)));
				// still room for one more hop in neighborhood exploration
				// the same is done for new contacts in RT (see onNewContact)
				if (viaPath.size() + packet.getPath(i).size() + 1  < discoveryRadius) {
					// first check for already queried neighbors to avoid unnecessary QueryRouteReq
					if (queriedNeighbors.find(neighborhoodContact.getKey()) == queriedNeighbors.end())
						sendGetPhysicalNeighbors(&neighborhoodContact);
				}

				if (usePathIDs)
					fwdTablePathID.addEntry(&neighborhoodContact, false);
			}
		}
	}
}


/* if duration is 0, this is a one shot timer */
short R2Kademlia::addTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback) {
	ASSERT(timerDuration.size() == timerMsg.size());
	ASSERT(timerDuration.size() < (size_type) std::numeric_limits<short int>::max());
	ASSERT(duration >= 0);
	if (duration > 0)
		timerDuration.push_back(duration);

	short int ntimerId(duration == 0 ? -1 : timerDuration.size()-1);
	cMessage *msg = new cMessage("TimerMsg", ntimerId);
	msg->setContextPointer( reinterpret_cast<void *>( new auto(callback) ) );

	if (duration > 0)
		timerMsg.push_back(msg);

	simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	//EV_TRACE << "scheduled timer message " << msg << ( (duration != 0) ? " with duration " : " after ")
	//	 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
	return ntimerId;
}


void R2Kademlia::stopTimer(short timerId)
{
	if (timerId > 0 and (size_type) timerId < timerMsg.size()) {
		cMessage* timermsg= timerMsg[timerId];
		timerMsg[timerId]= nullptr;
		// cancel potentially scheduled message (nothing happens if it was not)
		cancelEvent(timermsg);
		// remove callback function
		cbfunc_t* cbfunc_p(static_cast<cbfunc_t*>(timermsg->getContextPointer()));
		delete(cbfunc_p);
		timermsg->setContextPointer(nullptr);
		delete timermsg;
	}
}


// probe for closest neighbors
void R2Kademlia::doProbeNeighbors(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	if ( probeNeighborQueue.empty() )
	{
		// fill queue with current neighbors
		auto contacts= routingTable.getClosestContacts(myOverlayKey, routingTable.getSplitSize(), false);
		for (auto contactp : contacts)
			probeNeighborQueue.push_back(contactp->getKey());
	}

	// check for reachability of neighbors
	if (probeNeighborQueue.size()) {
		EV_TRACE << "probing for neighbor: " << probeNeighborQueue.front() << endl;
		sendFindNode(probeNeighborQueue.front(), true);
		probeNeighborQueue.pop_front();
	}
}


// check for path validity
void R2Kademlia::doProbeContacts(short timerId) {
	if (neighborTable.size() == 0)
		return; // node is isolated

	if ( probeContactQueue.empty() )
	{
		// fill queue with current neighbors
		auto contacts= routingTable.getAllValidContacts(true);
		for (auto contactp : contacts)
			probeContactQueue.push_back(contactp->getKey());
	}

	// check for reachability of neighbors
	if (probeContactQueue.size()) {
		EV_TRACE << "probing for contact: " << probeContactQueue.front() << endl;
		sendFindNode(probeContactQueue.front(), true);
		probeContactQueue.pop_front();
	}
}



void R2Kademlia::doProbe(short timerId) {
	if (neighborTable.size() == 0) return; // we are isolated
	OverlayKey probeKey;
	bool exact = false;
	uint32_t index = 0;
	switch (probeStrategy) {
		case 0:
			return;
		case 1:
			probeKey = OverlayKey::random(getRNG(0));
			break;
		case 2:
			probeKey = getKadObserver().getRandomNodeKey();
			break;
		case 3:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey;
			probeKey[myOverlayKey.getLength() - index] ^= 1;
			break;
		case 4:
			index = routingTable.getDepth();
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 5:
			index = std::min<size_type>(intuniform(1, 1 + routingTable.getDepth()*2),
					myOverlayKey.getLength());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		case 6:
			index = intuniform(1, 1 + routingTable.getDepth());
			probeKey = myOverlayKey.randomSuffix(myOverlayKey.getLength() - index, getRNG(0));
			break;
		default:
			throw cRuntimeError("unknown probe strategy %i, values from 0 to 6 are allowed");
	}
	if (probeKey == myOverlayKey) probeKey[0] ^= 1;
	sendFindNode(probeKey, exact);
}


void R2Kademlia::doProbeSelf(short timerId) {
	ASSERT(timerId >= 0);
	static const simtime_t maxDuration(300); // check every 5 minutes

	// exponential backoff double duration until we reached maxDuration
	if (timerDuration[timerId] < maxDuration)
	{ // double waiting time for next call until maxDuration is reached
		timerDuration[timerId] = std::min(2 * timerDuration[timerId], maxDuration);
	}

	if (neighborTable.size() == 0) return; // we are isolated
	bool exact= false;

	sendFindNode(myOverlayKey, exact);
}


void R2Kademlia::doReachabilityTest(short timerID) {
	if (neighborTable.size() == 0) return; // we are isolated
	const OverlayKey* key(nullptr);
	do {
		// during periods of failed links we want to test reachability for affected nodes
		if (getNetworkManager()->getFailedNodes().size() > 0) {
			auto nodeit= getNetworkManager()->getFailedNodes().cbegin();
			for (unsigned int index= intuniform(0, getNetworkManager()->getFailedNodes().size()-1); index > 0; index--)
				nodeit++;

			if (nodeit == getNetworkManager()->getFailedNodes().end())
				return;

			if (getNetworkManager()->getTopology().isActive(*nodeit) == false or
			    getNetworkManager()->getTopology().getNodeDegree(*nodeit) == 0) {
				if (getNetworkManager()->getFailedNodes().size() == 1) { // no other choice
					return;
				}
				else { // next try
					key= &myOverlayKey;
					continue;
				}
			}


			key = &getKadObserver().getNodeKey(*nodeit);
			// it does not make sense to use the own key if it is the only choice
			if (*key == myOverlayKey and getNetworkManager()->getFailedNodes().size() == 1)
				return;
		}
		else
			key = &getKadObserver().getRandomNodeKey();

	} while (*key == myOverlayKey);
	EV_TRACE << "Reachability Test to " << key->shortStr() << endl;
	sendFindNode(*key, true);
}


void R2Kademlia::countReceived(KadPacket& packet) {
	//EV_TRACE << "countReceived: tracked=" << packet.isTracked() << endl;
	receivedPackets++;
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
	packet.release();
	//EV_TRACE << "handleIfFitting tPacket " << static_cast<void*>(tPacket.get()) << endl;
	// delegate responsibility to handlePacket method
	handlePacket(std::move(tPacket));
	return true;
}

} //namespace
