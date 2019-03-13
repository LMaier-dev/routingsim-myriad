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

void KadObserver::initialize() {}
void KadObserver::finish() {
	nodes.clear();
}
void KadObserver::handleMessage(cMessage *msg) {
	delete msg;
}

void KadObserver::registerUp(const OverlayKey& key, nodeid_t owner) {
	ASSERT(nodes.left.count(key) == 0);
	ASSERT(nodes.right.count(owner) == 0);
	nodes.insert({key, owner});
}
void KadObserver::registerDown(const OverlayKey& key, nodeid_t owner) {
	auto it = nodes.right.find(owner);
	ASSERT(it != nodes.left.end());
	ASSERT(*it == key);
	nodes.right.erase(it);
}

static KadObserver* observer = NULL;

KadObserver& getKadObserver() {
	if (observer == NULL) {
		cModuleType* type = cModuleType::get(
				"routingsim.routing.R2Kademlia.KadObserver");
		cModule* mod = getSimulation()->getModuleByPath("<root>");
		observer = dynamic_cast<KadObserver*>(type->create("vrobserver", mod));
	}
	return *observer;
}
} // namespace
