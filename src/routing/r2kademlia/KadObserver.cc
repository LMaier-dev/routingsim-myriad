//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

/**
 * @file KadObserver.cc
 * @author Jakob Thomas
 */

#include "KadObserver.h"

namespace routingsim {
Define_Module(KadObserver);

/// singleton pointer
static KadObserver* observer = NULL;

void KadObserver::initialize() {
	ASSERT(observer == NULL || observer == this);
}

void KadObserver::finish() {
	nodes.clear();
	ASSERT(observer == this);
	observer = NULL;
}

void KadObserver::handleMessage(cMessage *msg) {
	delete msg;
}

void KadObserver::registerUp(const OverlayKey& key, nodeid_t owner) {
	ASSERT(not key.isUnspecified());
	ASSERT(owner != undefined_node);
	ASSERT(nodes.left.count(key) == 0);
	ASSERT(nodes.right.count(owner) == 0);
	nodes.insert({key, owner});
}
void KadObserver::registerDown(const OverlayKey& key, nodeid_t owner) {
	ASSERT(not key.isUnspecified());
	ASSERT(owner != undefined_node);
	auto it = nodes.right.find(owner);
	ASSERT(it != nodes.right.end());
	ASSERT(it->second == key);
	nodes.right.erase(it);
}

const OverlayKey& KadObserver::getRandomNodeKey() {
	auto it = nodes.left.begin();
	//TODO optimization: this is O(n), find something better!
	std::advance(it, intuniform(0, nodes.size() - 1));
	return it->first;
}

const OverlayKey& KadObserver::getClosestExistingKey(const OverlayKey& key) {
	ASSERT(nodes.size() > 0);
	auto itLower = nodes.left.lower_bound(key);
	if (itLower == nodes.left.end()) return (++itLower)->first;
	if (itLower->first == key) return key;
	auto itUpper = itLower--;
	return ((itLower->first ^ key) < (itUpper->first ^ key)) ? itLower->first : itUpper->first;
}

KadObserver& getKadObserver() {
	if (observer == NULL) {
		cModuleType* type = cModuleType::get(
				"routingsim.routing.r2kademlia.KadObserver");
		cModule* mod = getSimulation()->getModuleByPath("<root>");
		observer = dynamic_cast<KadObserver*>(type->create("KadObserver", mod));
	}
	return *observer;
}
} // namespace
