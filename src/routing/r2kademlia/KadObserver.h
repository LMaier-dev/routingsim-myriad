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

namespace routingsim {

class MessageStatistics;

/**
 * The R2/Kademlia Global Observer
 */
class KadObserver: public cSimpleModule {
protected:
	virtual void initialize() override;
	virtual void finish() override;
	virtual void handleMessage(cMessage *msg) override;

private:
	boost::bimap<OverlayKey, nodeid_t> nodes;

public:

	/// register a node with a key
	void registerUp(const OverlayKey& key, nodeid_t owner);
	/// deregister a node with a key
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
};

KadObserver& getKadObserver();

} // namespace

#endif
