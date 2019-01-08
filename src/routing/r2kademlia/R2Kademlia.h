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

namespace routingsim {

// packet types
using KadPacket_p = std::unique_ptr<KadPacket>;
using KadAnswer_p = std::unique_ptr<KadAnswer>;
using KadHello_p = std::unique_ptr<KadHello>;
using KadPing_p = std::unique_ptr<KadPing>;
using KadPong_p = std::unique_ptr<KadPong>;

/**
 * Implementation of R2Kademlia.
 * Routing with R/Kademlia...
 * TODO: sources
 * @author Jakob Thomas
 */
class R2Kademlia : public Protocol
{
  protected:
	/** @name Protocol interface */
	//@{
	/// called at initialization time of the Protocol
	void onInitialize();
	/// called when a new node is connected
	void onConnect(nodeid_t nid);
	/// called when a node is disconnected
	void onDisconnect(nodeid_t nid);
	/// on message
	void onMessage(NodePkt_p pkt);
	/// reset state of Protocol to initial state (Node Failure)
	void reset();
	//@}

	/** @name packet handling */
	//@{
	/// adds the neighbor to the neighbor table
	virtual void handlePacket(KadHello_p packet);
	virtual void handlePacket(KadPing_p packet){};
	virtual void handlePacket(KadAnswer_p packet){};
	/// fallback
	virtual void handlePacket(KadPacket_p packet);
	/// for packets with no remaining source route and target != myOverlayKey
	virtual void handleStrandedPacket(KadPacket_p packet){};
	//@}

	/** @name sending packets out*/
	//@{
	/**
	 * routes a packet to a neighbor based on the source route in the packet.
	 * uses the given NodePkt for encapsulation.
	 */
	void routePacket(KadPacket_p packet, NodePkt_p nodePkt);
	/// routes a packet to a neighbor based on the source route in the packet
	inline void routePacket(KadPacket_p packet)
		{ routePacket(std::move(packet), std::make_unique<NodePkt>()); }
	//@}

	/** @name policy foo */
	//@{
	void onMissingNeighbor(KadPacket_p packet, NodePkt_p pkt){};
	//@}
private:
	/// crafts a hello packet and encapsulates it in a NodePkt
	NodePkt_p makeHelloPacket();
	/// checks if the packet has some hops remaining in the source route field
	bool hasRemainingSourceRoute(const KadPacket& packet);
	/**
	 * Tries to cast packet to type T.
	 * If this succeeds, the method handlePacket(T) is called, the given pointer
	 * is invalidated and we return true.
	 * If the cast fails, nothing happends and we return false.
	 */
	template<typename T>
	bool handleIfFitting(KadPacket_p& packet);
private:
	OverlayKey myOverlayKey;
	NeighborTable neighborTable;
	static const int PAYLOAD_TYPE=5;
};

} //namespace

#endif // __ROUTINGSIM_R2Kademlia_H_
