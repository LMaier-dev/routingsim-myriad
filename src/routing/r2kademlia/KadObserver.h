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

#ifndef __ROUTINGSIM_KadObserver_H_
#define __ROUTINGSIM_KadObserver_H_

#include <boost/bimap.hpp>
#include "foundation.h"
#include "OverlayKey.h"
#include "R2Kademlia.h"
#include "foundation/NetworkManager.h"

namespace routingsim {
using cHistogramPtr = std::unique_ptr<cHistogram>;

class MessageStatistics;
struct MessageDeliveryInfo {
	uint hops = 0;
	uint overlayHops = 0;
	uint distance = 0;
	uint prefixDist = 0;
};
struct MessageDroppedInfo {
	uint hops = 0;
	uint overlayHops = 0;
	uint distance = 0;
	uint distanceLeft = 0;
	uint prefixDist = 0;
	uint prefixDistLeft = 0;
	uint reason = 0;
};

/**
 * The R2/Kademlia Global Observer
 */
class KadObserver: public cSimpleModule, public StatisticSample {
protected:
	virtual void initialize() override;
	virtual void finish() override;
	virtual void handleMessage(cMessage *msg) override;
	void recordStatistics();
	void recordMessageScalars();

private:
	boost::bimap<OverlayKey, nodeid_t> nodes;

	//
	unsigned int deliveredPackets = 0;
	unsigned int droppedPackets = 0;
	unsigned int registeredPackets = 0;

	unsigned int deliveredDataPackets = 0;
	unsigned int droppedDataPackets = 0;
	unsigned int registeredDataPackets = 0;
	
	enum dataType : uint8_t {hopCount=0, distance=1, prefixDist=2, overlayHops=3,
		stretchMul=4, stretchAdd=5, prefixPerHop=6, distanceLeft=7,
		prefixDistLeft=8, hopCountDropped=9, overlayHopsDropped=10,
		prefixDistDropped=11, distanceDropped=12};
	enum addDataType : uint8_t {deliveryRatioV=0, deliveredPacketsV=1, droppedPacketsV=2, roamingPacketsV=3};
	// histograms and vectors for statistics
	std::vector<cHistogramPtr> histograms;
	std::vector<std::unique_ptr<cOutVector>> outVectors;
	std::vector<std::unique_ptr<cOutVector>> addOutVectors;
	std::vector<MessageDeliveryInfo> messageDeliveryInfos;
	std::vector<MessageDroppedInfo> messageDroppedInfos;
	/// if true, histograms and averages are reset each sampling interval
	bool resetAfterSample;
	/// time when message recording should start
	simtime_t messageSampleTime;
	/// sets the max number of message delivery infos to be recorded
	size_type numMessageDeliveryInfos;
	/// sets the max number of message dropped infos to be recorded
	size_type numMessageDroppedInfos;

private:
	R2Kademlia& getR2Kademlia(Node* nodeMod);
	inline R2Kademlia& getR2Kademlia(nodeid_t nid) { return getR2Kademlia(getNetworkManager()->getTopology().getNodeModule(nid)); }
	inline R2Kademlia& getR2Kademlia(const OverlayKey& key) { return getR2Kademlia(getNodeId(key)); }
	std::set<nodeid_t> sampleNodes(int n);

public:

	/// register a node with a key
	void registerUp(const OverlayKey& key, nodeid_t owner);
	/// unregister a node with a key
	void registerDown(const OverlayKey& key, nodeid_t owner);

	/// returns the key of a given node id, if it is unknown std::out_of_range is thrown
	inline const OverlayKey& getNodeKey(nodeid_t owner) { return nodes.right.at(owner); };
	/// returns the node id of a given overlay key, if it is unknown std::out_of_range is thrown
	inline nodeid_t getNodeId(const OverlayKey& key) { return nodes.left.at(key); };
	/// returns true if a node with the given key is known
	inline bool exists(const OverlayKey& key) { return nodes.left.count(key) > 0; };

	/// returns the closest key that is assigned to a node
	const OverlayKey& getClosestExistingKey(const OverlayKey& key);
	/// returns the key of a random node (chosen uniformly at random)
	const OverlayKey& getRandomNodeKey();
	/// returns chooses a key at random and returns the closest existing key.
	/// note that this is not uniform over the nodes.
	const OverlayKey& getRandomExistingKey() { return getClosestExistingKey(OverlayKey::random(getRNG(0))); };
	/// returns the physical distance between the to nodes with the keys. the keys MUST exist!
	uint getPhysicalDistance(const OverlayKey& from, const OverlayKey& to);
	/// returns the number of other nodes that have this key in their routing table
	uint getPopularity(const OverlayKey& key);

	void registerPacket(KadPacket&);
	void registerPacketDropped(KadPacket&, const OverlayKey& current, const OverlayKey& target=OverlayKey::UNSPECIFIED_KEY);
	void registerPacketDelivered(KadPacket&);
	void registerPacketOverlayHop(KadPacket&);
	
	void registerPacket(KadDataPacket&);
	void registerPacketDropped(KadDataPacket&, const OverlayKey& current, const OverlayKey& target=OverlayKey::UNSPECIFIED_KEY);
	void registerPacketDelivered(KadDataPacket&);
	void registerPacketOverlayHop(KadDataPacket&);

	virtual void sampleStatistics (Statistics &stats) override;
};

KadObserver& getKadObserver();

} // namespace

#endif
