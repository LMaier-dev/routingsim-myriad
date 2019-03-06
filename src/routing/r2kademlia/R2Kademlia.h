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

#include "foundation.h"
#include "Protocol.h"
#include "StatisticSample.h"
#include "KadPackets_m.h"
#include "NeighborTable.h"
#include "KadRoutingTable.h"
#include "RPC.h"

namespace routingsim {

// packet types
using KadPacket_p = std::unique_ptr<KadPacket>;
using KadAnswer_p = std::unique_ptr<KadAnswer>;
using KadHello_p = std::unique_ptr<KadHello>;
using KadError_p = std::unique_ptr<KadError>;

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
	virtual void handlePacket(std::unique_ptr<KadQueryRouteReq> packet);
	virtual void handlePacket(std::unique_ptr<KadFindNodeReq> packet);
	virtual void handlePacket(KadAnswer_p packet);
	//@}

	/** @name sending packets out*/
	//@{
	/**
	 * forwards a packet to a neighbor based on the source route in the packet.
	 * uses the given NodePkt for encapsulation.
	 * This does not make any routing decision.
	 */
	void forwardPacket(KadPacket_p packet, NodePkt_p nodePkt);
	/// sends a recieved packet again after the source route was manipulated
	void resendPacket(KadPacket_p packet);
	/// routes a packet to a neighbor based on the source route in the packet
	void sendPacket(KadPacket_p packet) override;
	/// sends a probe request to the given contact
	void sendProbe(KadContact_p contact);
	/// sends a "get neighbors" call to the contact
	void sendGetNeighbors(KadContact_p contact);
	/// sends a find node call to find this node
	void sendFindNode(const OverlayKey& key);
	//@}

	/** @name policy foo */
	//@{
	void onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt);
	/** Called when an error packet is encountered, updates routing information */
	void onErrorPacket(KadError& error);
	/** called when a new contact is discovered */
	void onDiscovery(const OverlayKey& key, kad_path path);
	/** called when a path to a potentially new contact is discovered */
	void onPathDiscovery(const OverlayKey& key, kad_path path, const OverlayKey& via=OverlayKey::UNSPECIFIED_KEY);
	//@}
	virtual void sampleStatistics (Statistics &stats) override;
public:
	/** @name policy functions called by routing table */
	//@{
	/** called when there is space in a bucket and the bucket is not empty */
	virtual void onBucketSpace(KadBucket& bucket) override;
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) override;
	/**  called when a bucket needs to be refreshed */
	virtual void onBucketRefresh(KadBucket& bucket) override;
	/** called when a contact could be replaced by a new one */
	virtual void onBucketFull(KadBucket& bucket, KadContact_p contact) override;
	//@}
private:
	/** @name message generation */
	//@{
	/// crafts a hello packet and encapsulates it in a NodePkt
	NodePkt_p makeHelloPacket();
	/// crafts a packet of type T, sets the route to the given contact.
	template<typename T>
	std::unique_ptr<T> makePacket(KadContact_p contact, const char *name=nullptr);
	/// crafts a packet of type T, sets the route and nonce according to original
	template<typename T>
	std::unique_ptr<T> makeAnswer(KadPacket& original);
	//@}

	/** @name timer functions */
	//@{
	/// adds a timer to the timer vectors, sends a timer packet
	void addTimer(simtime_t duration, std::function<void()> callback);
	void findRandom();
	//@}

	/** @name private helpers */
	//@{
	/// does statistics on recieved packet
	void countRecieved(KadPacket& packet);
	/**
	 * sends an error packet to the sender of the given packet
	 * the original packet is encapsulated on the error packet
	 * @errorType the error type for the error packet
	 */
	void returnToSender(uint8_t errorType, KadPacket_p packet);
	/**
	 * Tries to cast packet to type T.
	 * If this succeeds, the method handlePacket(T) is called, the given pointer
	 * is invalidated and we return true.
	 * If the cast fails, nothing happends and we return false.
	 */
	template<typename T>
	bool handleIfFitting(KadPacket_p& packet);
	kad_path concatViaPath(const kad_path& viaPath, const OverlayKey& viaKey, const kad_path& path);
	//@}

	/** @name callbacks for rpc calls */
	//@{
	bool handleProbeResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleQueryRouteResponse(KadPacket_p answer, RPCState& rpcState);
	bool handleFindNodeResponse(KadPacket_p answer, RPCState& rpcState);
	//@}
private:
	/// the overlay key of this node
	OverlayKey myOverlayKey;
	NeighborTable neighborTable;
	KadRoutingTable routingTable;
	/// packet type used for Node::bind()
	static const int PAYLOAD_TYPE=5;
	/// number of contacts returned by find node response, called "k" in research papers
	uint findNodeNumber;
	/// radius of physical neighborhood discovery
	uint discoveryRadius;
	// statistics
	uint forwardedPackets = 0;
	uint droppedPackets = 0;
	uint sentPackets = 0;
	uint recievedPackets = 0;
	// timer data
	std::vector<simtime_t> timerDuration;
	std::vector<std::function<void()>> timerCallback;
	std::vector<cMessage *> timerMsg;
private:
	class PingRPCState;
};

template<typename T>
std::unique_ptr<T> safe_packet_cast(KadPacket_p& packet);
} //namespace

#endif // __ROUTINGSIM_R2Kademlia_H_
