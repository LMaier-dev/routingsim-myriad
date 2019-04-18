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
#include "NetworkManager.h"

namespace routingsim {
Define_Module(KadObserver);

/// singleton pointer
static KadObserver* observer = NULL;

void KadObserver::initialize() {
	ASSERT(observer == NULL || observer == this);
	getStatistics().addSample(this);
	hopCount = cHistogramPtr(getIntHistogram("hopCount"));
	distance = cHistogramPtr(getIntHistogram("distance"));
	prefixDist = cHistogramPtr(getIntHistogram("prefixDist"));
	overlayHops = cHistogramPtr(getIntHistogram("overlayHops"));
	stretchMul = std::make_unique<cHistogram>("stretchMul");
	stretchAdd = cHistogramPtr(getIntHistogram("stretchAdd"));
	prefixPerHop = std::make_unique<cHistogram>("prefixPerHop");

	distanceLeft = cHistogramPtr(getIntHistogram("distanceLeft"));
	prefixDistLeft = cHistogramPtr(getIntHistogram("prefixDistLeft"));

}

void KadObserver::finish() {
	hopCount->record();
	distance->record();
	prefixDist->record();
	overlayHops->record();
	stretchMul->record();
	stretchAdd->record();
	prefixPerHop->record();

	distanceLeft->record();
	prefixDistLeft->record();

	recordStatistics();

	nodes.clear();
	ASSERT(observer == this);
	observer = NULL;
}

void KadObserver::handleMessage(cMessage *msg) {
	delete msg;
}

void KadObserver::recordStatistics() {
	std::set<nodeid_t> nodeSample{};
	//TODO actually sample
	for (auto pair : nodes.right)
		nodeSample.insert(pair.first);
	cHistogram forwardedPackets{"forwardedPackets"};
	cHistogram nodeDegrees{"nodeDegrees"};
	cHistogram contacts{"routingTableContacts"};
	NetworkTopology& topo = getNetworkManager()->getTopology();
	for (nodeid_t nid : nodeSample) {
		R2Kademlia& kad = getR2Kademlia(nid);
		forwardedPackets.collect(kad.getForwardedPackets());
		nodeDegrees.collect(topo.getNodeDegree(nid));
		contacts.collect(kad.getRoutingTable().getNumContacts());
		kad.recordScalars();
	}
	forwardedPackets.record();
	nodeDegrees.record();
	contacts.record();
	recordScalar("simTime", simTime());
}

R2Kademlia& KadObserver::getR2Kademlia(Node* nodeMod) {
	cGate* gate = nodeMod->gate("toProtocols$o", 0);
	ASSERT(gate->isConnected());
	return *check_and_cast<R2Kademlia *>(gate->getPathEndGate()->getOwnerModule());
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

uint KadObserver::getPhysicalDistance(const OverlayKey& from, const OverlayKey& to) {
	ASSERT(exists(from));
	ASSERT(exists(to));
	return (uint) getNetworkManager()->getDistance(getNodeId(from), getNodeId(to));
}


uint KadObserver::getPopularity(const OverlayKey& key) {
	uint pop = 0;
	for (auto pair : nodes.right)
		if (getR2Kademlia(pair.first).getRoutingTable().getContact(key) != nullptr)
			pop++;
	return pop;
}


void KadObserver::registerPacket(KadPacket& packet) {
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	packet.markTracked();
	registeredPackets++;
}
void KadObserver::registerPacketDropped(KadPacket& packet, const OverlayKey& current, const OverlayKey& target) {
	if (not packet.isTracked()) return;
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	droppedPackets++;
	const OverlayKey& fromKey = packet.getSource();
	const OverlayKey& toKey = target.isUnspecified() ? packet.getTarget() : target;
	if (not exists(fromKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	MessageDroppedInfo info{};
	info.hops = packet.getProgress();
	info.overlayHops = packet.getOverlayHops();
	info.distance = getPhysicalDistance(fromKey, toKey);
	info.distanceLeft = getPhysicalDistance(current, toKey);
	info.prefixDist = fromKey.sharedPrefixLength(toKey);
	info.prefixDistLeft = current.sharedPrefixLength(toKey);
	info.reason = 0;
	prefixDistLeft->collect(info.prefixDistLeft);
	distanceLeft->collect(info.distanceLeft);

}
void KadObserver::registerPacketDelivered(KadPacket& packet) {
	if (not packet.isTracked()) return;
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	deliveredPackets++;
	const OverlayKey fromKey = packet.getSource();
	const OverlayKey toKey = packet.getTarget();
	if (not exists(fromKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	MessageDeliveryInfo info{};
	info.hops = packet.getProgress();
	info.distance = getPhysicalDistance(fromKey, toKey);
	info.prefixDist = fromKey.getLength() - fromKey.sharedPrefixLength(toKey);
	info.overlayHops = packet.getOverlayHops();
	hopCount->collect(info.hops);
	distance->collect(info.distance);
	stretchMul->collect(info.hops / (double)info.distance);
	stretchAdd->collect(info.hops - info.distance);
	prefixDist->collect(info.prefixDist);
	overlayHops->collect(info.overlayHops);
	prefixPerHop->collect(info.prefixDist / (double)info.overlayHops);
}
void KadObserver::registerPacketOverlayHop(KadPacket& packet) {
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	packet.countOverlayHop();
}

void KadObserver::sampleStatistics (Statistics &stats) {
	if (deliveredPackets > 0) {
		stats.value("hopCount", hopCount->getMean());
		stats.value("distance", distance->getMean());
		stats.value("prefixDist", prefixDist->getMean());
		stats.value("overlayHops", overlayHops->getMean());
		stats.value("stretchMul", stretchMul->getMean());
		stats.value("stretchAdd", stretchAdd->getMean());
		stats.value("prefixPerHop", prefixPerHop->getMean());

		stats.value("deliveryRatio", deliveredPackets/((double)deliveredPackets + droppedPackets));
	}

	if (droppedPackets > 0) {
		stats.value("distanceLeft", distanceLeft->getMean());
		stats.value("prefixDistLeft", prefixDistLeft->getMean());
	}

	stats.value("deliveredPackets", deliveredPackets);
	stats.value("droppedPackets", droppedPackets);
	stats.value("roamingPackets", registeredPackets - deliveredPackets - droppedPackets);

	if (resetAfterSample) {
		hopCount->clear();
		distance->clear();
		prefixDist->clear();
		overlayHops->clear();
		stretchMul->clear();
		stretchAdd->clear();
		prefixPerHop->clear();

		distanceLeft->clear();
		prefixDistLeft->clear();

		deliveredPackets = 0;
		droppedPackets = 0;
		registeredPackets = 0;
		recordScalar("lastReset", simTime());
	}
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
