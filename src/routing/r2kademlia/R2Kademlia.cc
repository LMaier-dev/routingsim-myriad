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
	// regularly send FindNodeReq to randomly chosen NodeIDs
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doProbe, this, ph::_1));
	// check whether other existing nodes can be reached (uses KadObserver)
	addTimer(probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doReachabilityTest, this, ph::_1));
	if (sendDataPackets)
	{
	  EV << "will send data packets after " << getSimulation()->getWarmupPeriod() << endl;
	  addTimer(getSimulation()->getWarmupPeriod()+probeInterval*1.5, probeInterval, std::bind(&R2Kademlia::doDataProbe, this, ph::_1));
	}

	WATCH(myOverlayKey);
	WATCH(neighborTable);
	WATCH_OBJ(routingTable);
	WATCH_OBJ(fwdTablePathID);

	// the following is not necessary, because onConnect will do this
	//sendToNeighbors(makeHelloPacket());
}


void R2Kademlia::onConnect( nodeid_t nid ) {
	sendToNode(nid, makeHelloPacket());
}


void R2Kademlia::onDisconnect( nodeid_t nid ) {
	//EV_TRACE << "onDisconnect " << nid << endl;
	// if this node is taken down, the packets arrive after the reset
	if(neighborTable.size() == 0) return;
	OverlayKey key = neighborTable.getOverlayKey(nid);
	ASSERT(not key.isUnspecified());
	ASSERT(neighborTable.getNodeId(key) == nid );

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
	scheduleRediscovery(key, key, myOverlayKey);

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

	if (packet->hasRemainingHops()) {
		// we are not the recipient
		if (pathSnooping) {
			// aka PATH OVERHEARING
			//TODO optimization use isPathInteresting

			// avoid learning about supposed physical Neighbors
			kad_path path_of_interest(packet->reversePath(packet->getSrcPath(1, packet->getProgress())));
			if (path_of_interest.size() > 0)
			{
				routingTable.addPath(packet->getSource(), path_of_interest, KadContact::PathProperty::indirect);
			}
			
			path_of_interest= packet->getSrcPath(packet->getProgress()+1, packet->getRouteArraySize()-1);
			if (path_of_interest.size() > 0)
			{
				routingTable.addPath(packet->getTarget(), path_of_interest, KadContact::PathProperty::indirect);
			}
		}
		
		//TODO ERROR OVERHEARING, more overhearing

		// add potential pathID for destination at this intermediate node
		switch (packet->getType())
		{
			case KAD_PathSetupReq: // allowed to take shortcuts
				// TODO send back response if path ID already installed and still valid
				applyShortcut(*packet);
				break;

			case KAD_PathSetupRsp:
				EV_TRACE << "forwarding PathSetupRsp, origin source path: <"
					 << packet->reversePath(packet->getSrcPath()) << endl;
				checkForPathInstallation(*packet);
				break;
		} // end switch
		forwardedPathSetup++;
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
		routingTable.addPath(packet->getSource(), packet->reversePath(packet->getIntermediatePath()), pathproperty);

		// these types may forward packets, so they count themselves
		if (handleIfFitting<KadFindNodeReq>(packet)) return;
		// these types never forwardpackets
		countReceived(*packet);
		if (handleIfFitting<KadQueryRouteReq>(packet)) return;
		if (handleIfFitting<KadUpdateRouteReq>(packet)) return;
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
	short timerId = msg->getKind();
	ASSERT(timerId >= 0);
	ASSERT((size_t)timerId < timerDuration.size());
	ASSERT(timerMsg[timerId] == msg.get());
	timerCallback[timerId](timerId);
	// reschedule timer if duration > 0, otherwise it was a one shot timer
	if (timerDuration[timerId] > 0)
	{
		simtime_t time = uniform(timerDuration[timerId]/2, timerDuration[timerId]/2*3);
		scheduleAt(simTime() + time, msg.release());
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
	EV << "received hello packet from " << packet->getSource().shortStr() << endl;

	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	//EV_TRACE << "neighbor table: "<< neighborTable << endl;

	// PNDiscovery should be send before any further messages to avoid missing neighbors
	KadContact neighborcontact(packet->getSource(), kad_path());
	sendDiscoveryReq(&neighborcontact);

	// try to add neighbor to routing table
	if (routingTable.addPath(packet->getSource(), kad_path(), KadContact::PathProperty::direct))
	{
		// send back PNDiscReq to get contacts from physical neighbor
		//TODO wait random time
		EV_TRACE << "adding neighbor successful" << endl;
	}
	else
		EV_TRACE << "adding neighbor failed (maybe was already there)" << endl;

	//EV_TRACE << "handlePacket: "<< static_cast<void*>(packet.get()) << endl;
}


void R2Kademlia::handlePacket(KadAnswer_p packet) {
	//EV_DEBUG << "received answer packet (for nonce=" << packet->getNonce() << ") from " << packet->getSource().shortStr() << endl;
	if (packet->getType() == KAD_Error) {
		onErrorPacket(*check_and_cast<KadError *>(packet.get()));
	}
	else if (packet->getType() == KAD_FindNodeRsp) {
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
	
	//TODO optimization: potential shortcut - we check if the target is a already neighbor
	// only allowed for exact and not strict routing

	const OverlayKey& nextHop = packet->getNextHop();
	nodeid_t nid = neighborTable.getNodeId(nextHop);

	EV_TRACE << "forwardPacket(): " << packet->getName() << " dest " << packet->getTarget().shortStr()
		 << " next hop: " << nextHop.shortStr() << " (" << nid << ')' << endl;

	if (nid != undefined_node) {
		nodePkt->encapsulate(packet.release());
		sendToNode(nid, std::move(nodePkt));
		forwardedPackets++;
	}
	else { // next hop not available
		// for Routing Updates and Rediscovery messages try to use overlay-based forwarding
		EV_TRACE << "forwardPacket(): path <";
		for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
			EV_TRACE << ' ' << packet->getRoute(i).shortStr();
		EV_TRACE << " > progress=" << packet->getProgress() << endl;

		const OverlayKey* finaldestination(nullptr);
		if (packet->getType() == KAD_UpdateRouteReq)
			finaldestination= &static_cast<const KadUpdateRouteReq*>(packet.get())->getDstNodeID();
		else
		if (packet->getType() == KAD_FindNodeReq and static_cast<const KadFindNodeReq*>(packet.get())->getNotVia() == true)
			finaldestination= &static_cast<const KadFindNodeReq*>(packet.get())->getDstNodeID();

		if (finaldestination != nullptr) {
			KadContactList ovlyneighbors= routingTable.getClosestContacts(*finaldestination, routingTable.getSplitSize());
			if (not ovlyneighbors.empty())
			{
				if (const KadContact& contact= chooseNextHop(ovlyneighbors, *finaldestination);
				    contact.isValid() and
				    (contact.getKey() ^ *finaldestination) < (myOverlayKey ^ *finaldestination))
				{ // we know a contact that is closer
					EV_TRACE << "forwardPacket(): after chooseNextHop" << endl;
					// intermediate target must also be part of the path
					kad_path newpath(contact.getPath());
					newpath.push_back(contact.getKey());

					packet->modRoute(newpath);
					EV_TRACE << "forwardPacket()/modRoute: contact "
						 << contact <<	" resulting path <";
					for (unsigned int i= 0; i < packet->getRouteArraySize(); i++)
						EV_TRACE << ' ' << packet->getRoute(i).shortStr();
					EV_TRACE << " > progress=" << packet->getProgress() << endl;

					const OverlayKey& nextHop = packet->getNextHop();
					nodeid_t nid = neighborTable.getNodeId(nextHop);

					if (nid != undefined_node) {
						EV_TRACE << "forwardPacket()/reroute: " << packet->getName() << " dest " << packet->getTarget().shortStr()
							 << " next hop: " << nextHop.shortStr() << " (" << nid << ')' << endl;
						nodePkt->encapsulate(packet.release());
						sendToNode(nid, std::move(nodePkt));
						forwardedPackets++;
						return;
					}
					else
						EV_TRACE << "forwardPacket()/reroute: cannot find alternative next hop" << endl;
				}
				else
					EV_TRACE << "forwardPacket()/reroute: cannot find alternative route via overlay" << endl;
			} //
			else
				EV_ERROR << "forwardPacket()/reroute: no contacts found" << endl;
		}

		// Trying to send to a next hop that is not a physical next hop
		// That will not fly...
		EV_ERROR << "forwardPacket(): could not find next hop!" << endl;
		onMissingNeighbor(std::move(packet), std::move(nodePkt));
	}
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
		const KadContact* contact = contacts.front();
		int dprefix = (contact->getKey() ^ target).log_2();
		size_t dproxMin = contact->getPathLength();
		if ((myOverlayKey ^ target).log_2() == dprefix) {
			// if we have no prefix progress,
			// we take the closest contact by XOR
			return *contact;
		}
		for (auto it = contacts.begin()+1; it != contacts.end(); it++) {
			if (((*it)->getKey() ^ target).log_2() > dprefix) break;
			if ((*it)->getPathLength() < dproxMin) {
				dproxMin = (*it)->getPathLength();
				contact = *it;
			}
		}
		return *contact;
	}
	else {
		// no proximity routing or only single contact
		//ASSERT(contacts.front()->getKey() != target); // this assertion seems to be wrong

		if (contacts.front()->getKey() == target)
		{
			EV_WARN << "Next hop is the target, number of contacts: " << contacts.size()
				 << "contact: " << *(contacts.front()) << endl;
		}

		return *(contacts.front());
	}
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
	sendPacket(makeAnswer<KadProbeRsp>(*packet));
}


bool R2Kademlia::handleProbeResponse(KadPacket_p answer, RPCState& rpcState) {
	if (rpcState.status != RPC_SUCCESS) return true;
	ASSERT(answer->getSource() == rpcState.original_packet->getTarget());
	std::unique_ptr<KadProbeRsp> probeRsp = safe_packet_cast<KadProbeRsp>(answer);
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
}


// Request k nearest Overlay Neighbors from contact 
void R2Kademlia::sendGetOverlayNeighbors(KadContact_p contact)
{
	EV_TRACE << "sending query route request (ON) to " << *contact << endl;
	auto req = makePacket<KadQueryRouteReq>(contact, "KadQueryRouteReq");
	req->setNeighborhood(0); // no physical neighbors
	req->setOverlayNeighbors(true); // get only overlay neighbors
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
}


// handle PNDiscReq
void R2Kademlia::handlePacket(std::unique_ptr<KadPNDiscReq> packet) {
	EV_TRACE << "received PNDiscReq from " << packet->getSource().shortStr() << endl;

	// since this is physical neighbor add it to our neighbortable
	neighborTable.addNeighbor(packet->getSourceId(), packet->getSource());
	EV_TRACE << "neighbor table: "<< neighborTable << endl;

	routingTable.addPath(packet->getSource(), kad_path(), KadContact::PathProperty::direct);

	// TODO currently only getting physical neighbors is implemented
	ASSERT(packet->getNeighborhood() == 1);
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadPNDiscRsp>(*packet);
	std::vector<OverlayKey> neighbors = neighborTable.getNeighbors();
	response->setContactArraySize(neighbors.size());
	response->setPathArraySize(neighbors.size());
	for (uint i = 0; i < neighbors.size(); i++)
		response->setContact(i, neighbors[i]);
	sendPacket(std::move(response));
}



void R2Kademlia::handlePacket(std::unique_ptr<KadQueryRouteReq> packet) {
	EV_TRACE << "received KadQueryRouteReq from " << packet->getSource().shortStr() << endl;

	//  packet->getContactsOnly() not implemented yet
	ASSERT(not packet->getContactsOnly());
	auto response = makeAnswer<KadQueryRouteRsp>(*packet);
	if (packet->getNeighborhood() == 1)
	{
		std::vector<OverlayKey> neighbors(neighborTable.getNeighbors());
		response->setContactArraySize(neighbors.size());
		response->setPathArraySize(neighbors.size());
		for (uint i = 0; i < neighbors.size(); i++)
			response->setContact(i, neighbors[i]);
	}
	else
	if (packet->getNeighborhood() == 63) // means to get full routing table
	{
		KadContactList allcontacts= routingTable.getAllContacts();
		response->setContactArraySize(allcontacts.size());
		response->setPathArraySize(allcontacts.size());
		for (uint i = 0; i < allcontacts.size(); i++)
		{
			response->setContact(i, allcontacts[i]->getKey());
			response->setPath(i, allcontacts[i]->getPath());
		}
	}

	if (packet->getOverlayNeighbors())
	{
		KadContactList ovlyneighbors= routingTable.getClosestContacts(myOverlayKey, routingTable.getSplitSize());

		response->setContactArraySize(ovlyneighbors.size());
		response->setPathArraySize(ovlyneighbors.size());
		for (uint i = 0; i < ovlyneighbors.size(); i++)
		{
			response->setContact(i, ovlyneighbors[i]->getKey());
			response->setPath(i, ovlyneighbors[i]->getPath());
		}

	}

	sendPacket(std::move(response));
}


bool R2Kademlia::handlePNDiscResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadPNDiscRsp>(answer);
	EV_TRACE << "PNDiscRsp received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	readRTableRsp(std::move(response));
	return true;
}

	
bool R2Kademlia::handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState)
{
	if (rpcState.status != RPC_SUCCESS) return true;
	auto response = safe_packet_cast<KadQueryRouteRsp>(answer);
	EV_TRACE << "received " << response->getContactArraySize() << " neighbors from " << response->getSource().shortStr() << endl;
	readRTableRsp(std::move(response));
	return true;
}


// FIND NODE
void R2Kademlia::sendFindNode(const OverlayKey& key, bool exact, const OverlayKey* notvia_unreachable_node, const OverlayKey* notvia_failednode_neighbor) {
	bool is_rediscovery(notvia_unreachable_node and  notvia_failednode_neighbor);
	EV_TRACE << "sendFindNode w/ " << key.shortStr() << (exact ? " (NID)" : (key == myOverlayKey ? " (JOIN)" : " (RID)")) // Node ID or Resource ID?
		 << (is_rediscovery ? " (Rediscovery)" : "") <<  endl;


	//ASSERT(contacts.size() > 0);
	if (auto contacts = routingTable.getClosestContacts(key, findNodeNumber); contacts.size() > 0)
	{
		auto maxFanOut= is_rediscovery ? rediscoveryFanOutFactor : 1;
		// use several contacts to send multiple Rediscovery requests in parallel
		for (auto fanout= 0; fanout < maxFanOut and contacts.size() > 0; fanout++)
		{
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
			if (is_rediscovery)
			{ // this is an immediate rediscovery packet
				ASSERT(exact);
				packet->setNotVia(true);
				packet->setUnreachableNode(*notvia_unreachable_node);
				packet->setDetectingNode(*notvia_failednode_neighbor);
				sentRediscovery++;
				EV_TRACE << "Rediscovery sending ... msg id=" << packet->getId()
					 << " via " << contact << " fanout: " << fanout << endl;
			}
			EV_TRACE << "sending KadFindNodeReq id=" << packet->getId() << endl;
			sendRpcCall(
				std::move(packet),
				std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
				std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
				std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
				);
			// remove the used element and try again (might not be very efficient, but the list is short)
			contacts.erase(std::find(contacts.begin(),contacts.end(),&contact));
		}

	}
	else
	{ // no contacts to choose from (usually only possible for isolated nodes)
		EV_WARN << "no contacts found, cannot send FindNodeReq" << endl;
	}
}


void R2Kademlia::handlePacket(std::unique_ptr<KadFindNodeReq> packet) {
	OverlayKey& query = packet->getDstNodeID(); // get the destination ID

	EV_TRACE << "KadFindNodeReq (id=" << packet->getId() << ", nonce=" << packet->getNonce() << ") received query for: " << query.shortStr() << " isTracked=" << packet->isTracked() << endl;
	bool is_rediscovery= false;
	// check for presence of "not via" setting
	if (packet->getNotVia())
	{
		EV_TRACE << "FindNodeReq contains not via <"
			 << packet->getUnreachableNode().shortStr() << ' '
			 << packet->getDetectingNode().shortStr() << '>' << endl;
		// we should invalidate all contacts with that link in the path to them before answering
		routingTable.invalidateContacts(packet->getUnreachableNode(), packet->getDetectingNode());
		is_rediscovery= true;
	}
	
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList contacts = routingTable.getClosestContacts(query, contactsLen);

	if (contacts.size() == 0)
	{
		// send back Route Failure: no route toward destination known
		// the only contact we know is the source
		EV_TRACE << "no contacts closer to destination found" << endl;
		countDropped(*packet, query, routeFailureDeadEnd);
		returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

		if (is_rediscovery)
			droppedRediscovery++;

		return;
	}

	// precondition here: routing table has at least one entry
	if (query != myOverlayKey) { // someone else routing via us

		// the first contact is closest to query key
		// if it is the source, we would route it back in a loop
		// (may be also the case if a node tries to join and looks up its own node ID)
		if (contacts.front()->getKey() == packet->getSource()) {
			if (contacts.size() == 1) {
				// the only contact we know is the source
				EV_TRACE << "only one contact known: the source" << endl;
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

		/*
		EV_TRACE << "Found contacts:";
		for (const auto& thiscontact : contacts)
		{
			EV_TRACE << *thiscontact;
		}
		EV_TRACE << endl;
		*/
		
		// if contact is closer to destination than us, it's ok
		if ((contacts.front()->getKey() ^ query) < (myOverlayKey ^ query)
				and contacts.front()->getPathLength() < packet->getHopLimit()) {
			// we have found a closer contact that is close enough
			// => forward request
			const KadContact& contact = chooseNextHop(contacts, query);
			EV_TRACE << "KadFindNodeReq received query for: " << packet->getDstNodeID().shortStr() << endl << " next hop is " << contact << endl;
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
					 << " own distance=" << (myOverlayKey ^ query).log_2() << endl;
				if ((contact.getKey() ^ query).log_2() > (myOverlayKey ^ query).log_2()) {
					EV_WARN << "no prefix progress made - dropping request" << endl;
				}
				// since we need an exact match and cannot make any progress, we are lost here
				countDropped(*packet, query, routeFailureDeadEnd);
				//TODO: if we find a closer contact too far away, we should send HOP LIMIT EXCEEDED
				returnToSender(KAD_RouteFailureDeadEnd, std::move(packet));

				if (is_rediscovery)
					droppedRediscovery++;
				return;

			} // if exact flag is set

			// otherwise it's ok to report what we know

		} // end else
	}
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
	if (rpcState.status != RPC_SUCCESS) return true;
	auto answer = safe_packet_cast<KadFindNodeRsp>(answer_);
	ASSERT(answer->getContactArraySize() == answer->getPathArraySize());
	ASSERT(rpcState.original_packet.get() != nullptr);
	EV_TRACE << "received FindNodeRsp for " << check_and_cast<KadFindNodeReq *>(rpcState.original_packet.get())->getDstNodeID().shortStr();
	EV_TRACE << " from " << answer->getSource().shortStr() << " with " << answer->getContactArraySize() << " contacts" << endl;
	readRTableRsp(std::move(answer));
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
	
	switch (error.getErrorType()) {
		// the following errors indicate a problem with the route to the contact
		case KAD_SegmentFailure:
		case KAD_NodeUnreachable:
		case KAD_PathIDUnknown:
		{ // trigger rediscovery
			ASSERT(error.getEncapsulatedPacket() != nullptr);
			KadPacket* original = check_and_cast<KadPacket *>(error.getEncapsulatedPacket());
			ASSERT(original->getNonce() == error.getNonce());
			ASSERT(original->getSource() == myOverlayKey);

			auto affectedContact= routingTable.getContact(original->getTarget());
			if (affectedContact)
			{
				if (affectedContact->isValid())
				{
					EV_INFO << "Invalidating contact " << affectedContact->getKey().shortStr() << " because of error packet" << endl;
					routingTable.setContactInvalid(affectedContact->getKey());
					// find out the failed link
					auto reportingNode = error.getSource();
					const auto originalPath= original->getSrcPath();
					// find reporting node in the source path
					auto it= find(originalPath.begin(),originalPath.end(),reportingNode);
					// the following should be true, otherwise we are getting 
					// an error from some off path node
					ASSERT(it != originalPath.end());
					// the next hop is the one that failed
					auto failedNextHop = (++it != originalPath.end()) ? *it : reportingNode ; 

					scheduleRediscovery(affectedContact->getKey(), reportingNode, failedNextHop);
				}
				else
				{ // maybe already invalid or rediscovering
					if (affectedContact->isRediscovering())
					{ // must check whether this is an error message for the rediscovery attempt
						//EV_TRACE << "Contact " << affectedContact->getKey().shortStr()
						//	 << " already in state rediscovering" << endl;
						auto findnodereq= dynamic_cast<const KadFindNodeReq*>(original);
						if (findnodereq == nullptr or findnodereq->getNotVia() == false) {
							// failed packet was not a rediscovery packet
							break;
						}
						else {
							EV_INFO << "Rediscovery failed - removing invalid contact "
								<< affectedContact->getKey().shortStr()
								<< " because of error packet" << endl;

							routingTable.removeContact(affectedContact->getKey());
						}
					}

					// invalid contact should be scheduled for rediscovery

				}
			}
			// else contact may have been removed meanwhile (e.g., by contact replacement)
		}
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

	EV_TRACE << "onNewContact(" << contact->getKey().shortStr() << ") called" << endl;

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
		// request info about overlay neighbors of nodes in our deepest bucket
		if (routingTable.isInLastBucket(contact->getKey()))
		{
			//EV_TRACE << "New contact is in deepest bucket, will ask for its routing table" << endl;
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
}


void R2Kademlia::onImprovement(const KadContact& contact) {
	// path got improved for this contact -> potentially send UpdateRouteReq
	EV_TRACE << "onImprovement(" << contact.getKey().shortStr() << ") called" << endl;

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
		scheduleUpdate(contact, KAD_Announce);
	}
	else
		scheduleUpdate(contact, KAD_Change);
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

				EV_TRACE << " new=" << packet.getSrcPath() << endl;
			}
		}
		else
			EV_WARN << "Contact in non valid state: " << *knownContact;
	}
}


void R2Kademlia::checkForPathInstallation(KadPacket& packet)
{
	if ( not(usePathIDs) )
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
	ASSERT(contacts.size() > 0);
	packet.setContactArraySize(contacts.size());
	if (includePaths) {
		packet.setPathArraySize(contacts.size());
	}
	for (size_t i = 0; i < contacts.size(); i++) {
		packet.setContact(i, contacts[i]->getKey());
		if (includePaths) {
			packet.setPath(i, contacts[i]->getPath());
		}
	}
}


void R2Kademlia::readRTableRsp(std::unique_ptr<KadRTableRsp> packet) {
	ASSERT(packet->getContactArraySize() == packet->getPathArraySize());
	kad_path viaPath = packet->reversePath(packet->getIntermediatePath());
	viaPath.push_back(packet->getSource());
	//routingTable.shortenPath(viapath);

	for (size_t i = 0; i < packet->getContactArraySize(); i++) {
		// TODO optimization: isPathInteresting
		if ( not(routingTable.addPath(packet->getContact(i), std::move(packet->getPath(i)), viaPath)) ) {
			// even if contact was not interesting for RT, add it as PID entry if within discovery radius
			if ( viaPath.size() + packet->getPath(i).size() < discoveryRadius )
			{
				KadContact neighborhoodContact(packet->getContact(i),
							       routingTable.concatViaPath(viaPath, packet->getPath(i)));
				fwdTablePathID.addEntry(&neighborhoodContact, false);
			}
		}
	}
}


/* if duration is 0, this is a one shot timer */
void R2Kademlia::addTimer(simtime_t initialOffset, simtime_t duration, std::function<void(short timerId)> callback) {
	ASSERT(timerDuration.size() == timerCallback.size());
	ASSERT(timerDuration.size() < std::numeric_limits<long unsigned int>::max());
	ASSERT(duration >= 0);
	timerDuration.push_back(duration);
	timerCallback.push_back(callback);
	cMessage *msg = new cMessage("TimerMsg", timerDuration.size()-1);
	timerMsg.push_back(msg);
	simtime_t scatter = uniform(0, duration);
	scheduleAt(simTime() + initialOffset - duration/2 + scatter, msg);
	//EV_TRACE << "scheduled timer message " << msg << ( (duration != 0) ? " with duration " : " after ")
	//	 <<   ((duration != 0) ? duration : initialOffset) <<  endl;
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
	const OverlayKey* key;
	do {
		if (getNetworkManager()->getFailedNodes().size() > 0) {
			auto nodeit= getNetworkManager()->getFailedNodes().cbegin();
			for (unsigned int index= intuniform(0, getNetworkManager()->getFailedNodes().size()-1); index > 0; index--)
				nodeit++;

			key = &getKadObserver().getNodeKey(*nodeit);
			// it does not make sense to use the own key if it is the only choice
			if (*key == myOverlayKey and getNetworkManager()->getFailedNodes().size() == 1)
				return;
		}
		else
			key = &getKadObserver().getRandomNodeKey();

	} while (*key == myOverlayKey);

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
