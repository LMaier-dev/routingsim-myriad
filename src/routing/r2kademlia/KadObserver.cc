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
	if (getEnvir()->getConfig()->getConfigValue("sim-time-limit") != nullptr) {
		simTimeLimit= SimTime::parse(getEnvir()->getConfig()->getConfigValue("sim-time-limit"));
	}
	getStatistics().addSample(this);

	resetAfterSample = par("resetAfterSample").boolValue();
	messageSampleTime = (simtime_t) par("messageSampleTime").doubleValue();
	numMessageDeliveryInfos = par("messageDeliverySample").intValue();
	numMessageDroppedInfos = par("messageDroppedSample").intValue();

	histograms.resize(numDataTypes);
	histograms[hopCount] = cHistogramPtr(getIntHistogram("hopCount"));
	histograms[distance] = cHistogramPtr(getIntHistogram("distance"));
	histograms[prefixDist] = cHistogramPtr(getIntHistogram("prefixDist"));
	histograms[overlayHops] = cHistogramPtr(getIntHistogram("overlayHops"));
	histograms[stretchMul] = std::make_unique<cHistogram>("stretchMul");
	histograms[stretchAdd] = cHistogramPtr(getIntHistogram("stretchAdd"));
	histograms[prefixPerHop] = std::make_unique<cHistogram>("prefixPerHop");
	histograms[distanceLeft] = cHistogramPtr(getIntHistogram("distanceLeft"));
	histograms[prefixDistLeft] = cHistogramPtr(getIntHistogram("prefixDistLeft"));
	histograms[hopCountDropped] = cHistogramPtr(getIntHistogram("hopCountDropped"));
	histograms[overlayHopsDropped] = cHistogramPtr(getIntHistogram("overlayHopsDropped"));
	histograms[prefixDistDropped] = cHistogramPtr(getIntHistogram("prefixDistDropped"));
	histograms[distanceDropped] = cHistogramPtr(getIntHistogram("distanceDropped"));
	histograms[hopCountData] = cHistogramPtr(getIntHistogram("hopCountData"));
	histograms[distanceData] = cHistogramPtr(getIntHistogram("distanceData"));
	histograms[prefixDistData] = cHistogramPtr(getIntHistogram("prefixDistData"));
	histograms[overlayHopsData] = cHistogramPtr(getIntHistogram("overlayHopsData"));
	histograms[stretchMulData] = std::make_unique<cHistogram>("stretchMulData");
	histograms[stretchAddData] = cHistogramPtr(getIntHistogram("stretchAddData"));
	histograms[prefixPerHopData] = std::make_unique<cHistogram>("prefixPerHopData");
	histograms[distanceLeftData] = cHistogramPtr(getIntHistogram("distanceLeftData"));
	histograms[prefixDistLeftData] = cHistogramPtr(getIntHistogram("prefixDistLeftData"));
	histograms[hopCountDroppedData] = cHistogramPtr(getIntHistogram("hopCountDroppedData"));
	histograms[overlayHopsDroppedData] = cHistogramPtr(getIntHistogram("overlayHopsDroppedData"));
	histograms[prefixDistDroppedData] = cHistogramPtr(getIntHistogram("prefixDistDroppedData"));
	histograms[distanceDroppedData] = cHistogramPtr(getIntHistogram("distanceDroppedData"));
	histograms[sentRouteUpdates] = cHistogramPtr(getIntHistogram("sentRouteUpdates"));
	histograms[receivedRouteUpdates] = cHistogramPtr(getIntHistogram("receivedRouteUpdates"));
	histograms[sentPathSetup] = cHistogramPtr(getIntHistogram("sentPathSetup"));
	histograms[receivedPathSetup] = cHistogramPtr(getIntHistogram("receivedPathSetup"));
	histograms[forwardedPathSetup] = cHistogramPtr(getIntHistogram("forwardedPathSetup"));
	histograms[stretchMulAns] = std::make_unique<cHistogram>("stretchMulAns");
	histograms[stretchAddAns] = cHistogramPtr(getIntHistogram("stretchAddAns"));
	histograms[stretchMulLater] = std::make_unique<cHistogram>("stretchMulLater");
	histograms[stretchAddLater] = cHistogramPtr(getIntHistogram("stretchAddLater"));

	
	outVectors.resize(histograms.size());
	outVectors[hopCount] = std::make_unique<cOutVector>("hopCount");
	outVectors[distance] = std::make_unique<cOutVector>("distance");
	outVectors[prefixDist] = std::make_unique<cOutVector>("prefixDist");
	outVectors[overlayHops] = std::make_unique<cOutVector>("overlayHops");
	outVectors[stretchMul] = std::make_unique<cOutVector>("stretchMul");
	outVectors[stretchAdd] = std::make_unique<cOutVector>("stretchAdd");
	outVectors[prefixPerHop] = std::make_unique<cOutVector>("prefixPerHop");
	outVectors[distanceLeft] = std::make_unique<cOutVector>("distanceLeft");
	outVectors[prefixDistLeft] = std::make_unique<cOutVector>("prefixDistLeft");
	outVectors[hopCountDropped] = std::make_unique<cOutVector>("hopCountDropped");
	outVectors[overlayHopsDropped] = std::make_unique<cOutVector>("overlayHopsDropped");
	outVectors[prefixDistDropped] = std::make_unique<cOutVector>("prefixDistDropped");
	outVectors[distanceDropped] = std::make_unique<cOutVector>("distanceDropped");
	outVectors[hopCountData] = std::make_unique<cOutVector>("hopCountData");
	outVectors[distanceData] = std::make_unique<cOutVector>("distanceData");
	outVectors[prefixDistData] = std::make_unique<cOutVector>("prefixDistData");
	outVectors[overlayHopsData] = std::make_unique<cOutVector>("overlayHopsData");
	outVectors[stretchMulData] = std::make_unique<cOutVector>("stretchMulData");
	outVectors[stretchAddData] = std::make_unique<cOutVector>("stretchAddData");
	outVectors[prefixPerHopData] = std::make_unique<cOutVector>("prefixPerHopData");
	outVectors[distanceLeftData] = std::make_unique<cOutVector>("distanceLeftData");
	outVectors[prefixDistLeftData] = std::make_unique<cOutVector>("prefixDistLeftData");
	outVectors[hopCountDroppedData] = std::make_unique<cOutVector>("hopCountDroppedData");
	outVectors[overlayHopsDroppedData] = std::make_unique<cOutVector>("overlayHopsDroppedData");
	outVectors[prefixDistDroppedData] = std::make_unique<cOutVector>("prefixDistDroppedData");
	outVectors[distanceDroppedData] = std::make_unique<cOutVector>("distanceDroppedData");
	outVectors[sentRouteUpdates] = std::make_unique<cOutVector>("sentRouteUpdates");
	outVectors[receivedRouteUpdates] = std::make_unique<cOutVector>("receivedRouteUpdates");
	outVectors[sentPathSetup] = std::make_unique<cOutVector>("sentPathSetup");
	outVectors[receivedPathSetup] = std::make_unique<cOutVector>("receivedPathSetup");
	outVectors[forwardedPathSetup] = std::make_unique<cOutVector>("forwardedPathSetup");
	outVectors[stretchMulAns] = std::make_unique<cOutVector>("stretchMulAns");
	outVectors[stretchAddAns] = std::make_unique<cOutVector>("stretchAddAns");
	outVectors[stretchMulLater] = std::make_unique<cOutVector>("stretchMulLater");
	outVectors[stretchAddLater] = std::make_unique<cOutVector>("stretchAddLater");

	addOutVectors.resize(numAddDataTypes);
	addOutVectors[deliveryRatioV] = std::make_unique<cOutVector>("deliveryRatio");
	addOutVectors[deliveredPacketsV] = std::make_unique<cOutVector>("deliveredPackets");
	addOutVectors[droppedPacketsV] = std::make_unique<cOutVector>("droppedPackets");
	addOutVectors[roamingPacketsV] = std::make_unique<cOutVector>("roamingPackets");
	addOutVectors[deliveryRatioDataV] = std::make_unique<cOutVector>("deliveryRatioData");
	addOutVectors[deliveredDataPacketsV] = std::make_unique<cOutVector>("deliveredDataPackets");
	addOutVectors[droppedDataPacketsV] = std::make_unique<cOutVector>("droppedDataPackets");
	addOutVectors[roamingDataPacketsV] = std::make_unique<cOutVector>("roamingDataPackets");
}

void KadObserver::finish() {
	for (auto& h : histograms)
		h->record();
	recordStatistics();
	recordMessageScalars();

	nodes.clear();
	histograms.clear();
	outVectors.clear();
	addOutVectors.clear();
	ASSERT(observer == this);
	observer = NULL;
}

void KadObserver::handleMessage(cMessage *msg) {
	delete msg;
}

std::set<nodeid_t> KadObserver::sampleNodes(int n) {
	if (n < 0 or (size_type) n > nodes.size())
		n = nodes.size();
	std::set<nodeid_t> sample{};
	NetworkTopology& topo = getNetworkManager()->getTopology();
	// use all nodes if n larger than 80% of all nodes (then sampling is not useful anymore)
	if (n > nodes.size() * 0.8) {
		for (auto& pair : nodes.right)
			sample.insert(pair.first);
	}
	while (sample.size() < (size_type) n) {
		nodeid_t nid = topo.getRandomNode(getRNG(0));
		ASSERT(nodes.right.count(nid) == 1);
		sample.insert(nid);
	}
	while (sample.size() > (size_type) n)
		sample.erase(topo.getRandomNode(getRNG(0)));

	return sample;
}

void KadObserver::recordStatistics() {
	for (nodeid_t nid : sampleNodes(par("nodeSample").intValue()))
		getR2Kademlia(nid).recordScalars();
	for (nodeid_t nid : sampleNodes(par("routingTableSample").intValue()))
		getR2Kademlia(nid).recordRoutingTable();
	recordScalar("simTime", simTime());
}


void KadObserver::recordMessageScalars() {
	cModuleType* modType = cModuleType::get("routingsim.routing.r2kademlia.Dummy");
	std::string messageDeliveredStr{"messageDelivered"};
	uint i = 0;
	for (auto& info : messageDeliveryInfos){
		cModule* mod = modType->create((messageDeliveredStr + std::to_string(++i)).c_str(), this);
		mod->recordScalar("hopCount", info.hops);
		mod->recordScalar("overlayHops", info.overlayHops);
		mod->recordScalar("distance", info.distance);
		mod->recordScalar("prefixDist", info.prefixDist);
	}
	i = 0;
	std::string messageDroppedStr{"messageDropped"};
	for (auto& info : messageDroppedInfos){
		cModule* mod = modType->create((messageDroppedStr + std::to_string(++i)).c_str(), this);
		mod->recordScalar("hopCountDropped", info.hops);
		mod->recordScalar("overlayHopsDropped", info.overlayHops);
		mod->recordScalar("distanceDropped", info.distance);
		mod->recordScalar("distanceLeft", info.distanceLeft);
		mod->recordScalar("prefixDistDropped", info.prefixDist);
		mod->recordScalar("prefixDistLeft", info.prefixDistLeft);
		mod->recordScalar("reasonDropped", info.reason);
	}

	std::string dataDeliveredStr{"dataDelivered"};
	i = 0;
	for (auto& info : dataDeliveryInfos){
		cModule* mod = modType->create((dataDeliveredStr + std::to_string(++i)).c_str(), this);
		mod->recordScalar("hopCount", info.hops);
		mod->recordScalar("overlayHops", info.overlayHops);
		mod->recordScalar("distance", info.distance);
		mod->recordScalar("prefixDist", info.prefixDist);
	}
	i = 0;
	std::string dataDroppedStr{"dataDropped"};
	for (auto& info : dataDroppedInfos){
		cModule* mod = modType->create((dataDroppedStr + std::to_string(++i)).c_str(), this);
		mod->recordScalar("hopCountDropped", info.hops);
		mod->recordScalar("overlayHopsDropped", info.overlayHops);
		mod->recordScalar("distanceDropped", info.distance);
		mod->recordScalar("distanceLeft", info.distanceLeft);
		mod->recordScalar("prefixDistDropped", info.prefixDist);
		mod->recordScalar("prefixDistLeft", info.prefixDistLeft);
		mod->recordScalar("reasonDropped", info.reason);
	}

}

R2Kademlia& KadObserver::getR2Kademlia(Node* nodeMod) {
	cGate* gate = nodeMod->gate("toProtocols$o", 0);
	ASSERT(gate->isConnected());
	return *check_and_cast<R2Kademlia *>(gate->getPathEndGate()->getOwnerModule());
}


const R2Kademlia& KadObserver::getR2Kademlia(const Node* nodeMod) const {
	const cGate* gate = nodeMod->gate("toProtocols$o", 0);
	ASSERT(gate->isConnected());
	return *check_and_cast<const R2Kademlia *>(gate->getPathEndGate()->getOwnerModule());
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

const OverlayKey& KadObserver::getRandomNodeKey() const {
	auto it = nodes.left.begin();
	//TODO optimization: this is O(n), find something better!
	std::advance(it, intuniform(0, nodes.size() - 1));
	return it->first;
}

const OverlayKey& KadObserver::getClosestExistingKey(const OverlayKey& key) const {
	ASSERT(nodes.size() > 0);
	auto itLower = nodes.left.lower_bound(key);
	if (itLower == nodes.left.end()) return (++itLower)->first;
	if (itLower->first == key) return key;
	auto itUpper = itLower--;
	return ((itLower->first ^ key) < (itUpper->first ^ key)) ? itLower->first : itUpper->first;
}

uint KadObserver::getPhysicalDistance(const OverlayKey& from, const OverlayKey& to) const {
	ASSERT(exists(from));
	ASSERT(exists(to));
	return (uint) getNetworkManager()->getDistance(getNodeId(from), getNodeId(to));
}


uint KadObserver::getPopularity(const OverlayKey& key) const {
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


void KadObserver::registerPacket(KadDataPacket& packet) {
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	packet.markTracked();
	registeredPackets++;
}


void KadObserver::registerPacketDropped(KadPacket& packet, const OverlayKey& current, const OverlayKey& target, uint8_t reason) {
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
	info.reason = reason;
	histograms[hopCountDropped]->collect(info.hops);
	histograms[overlayHopsDropped]->collect(info.overlayHops);
	histograms[distanceDropped]->collect(info.distance);
	histograms[distanceLeft]->collect(info.distanceLeft);
	histograms[prefixDistDropped]->collect(info.prefixDist);
	histograms[prefixDistLeft]->collect(info.prefixDistLeft);

	if (simTime() > messageSampleTime and messageDroppedInfos.size() < numMessageDroppedInfos)
		messageDroppedInfos.push_back(info);

}


void KadObserver::registerPacketDropped(KadDataPacket& packet, const OverlayKey& current, const OverlayKey& target, uint8_t reason) {
	if (not packet.isTracked()) return;
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	droppedDataPackets++;
	const OverlayKey& fromKey = packet.getSrcNodeID();
	const OverlayKey& toKey = target.isUnspecified() ? packet.getDstNodeID() : target;
	if (not exists(fromKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	MessageDroppedInfo info{};
	info.hops = getR2Kademlia(fromKey).getHopLimitDefault() - packet.getHopLimit();
	info.overlayHops = packet.getOverlayHops();
	info.distance = getPhysicalDistance(fromKey, toKey);
	info.distanceLeft = getPhysicalDistance(current, toKey);
	info.prefixDist = fromKey.sharedPrefixLength(toKey);
	info.prefixDistLeft = current.sharedPrefixLength(toKey);
	info.reason = reason;
	histograms[hopCountDroppedData]->collect(info.hops);
	histograms[overlayHopsDroppedData]->collect(info.overlayHops);
	histograms[distanceDroppedData]->collect(info.distance);
	histograms[distanceLeftData]->collect(info.distanceLeft);
	histograms[prefixDistDroppedData]->collect(info.prefixDist);
	histograms[prefixDistLeftData]->collect(info.prefixDistLeft);

	if (simTime() > messageSampleTime and dataDroppedInfos.size() < numMessageDroppedInfos)
		dataDroppedInfos.push_back(info);

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

	//EV_TRACE << "packetDelivered: hops=" << info.hops << " distance=" << info.distance << endl;

	ASSERT(info.overlayHops > 0);
	histograms[hopCount]->collect(info.hops);
	histograms[distance]->collect(info.distance);
	histograms[stretchMul]->collect(info.hops / (double)info.distance);
	histograms[stretchAdd]->collect(info.hops - info.distance);
	histograms[prefixDist]->collect(info.prefixDist);
	histograms[overlayHops]->collect(info.overlayHops);
	histograms[prefixPerHop]->collect(info.prefixDist / (double)info.overlayHops);
	if (simTime() > messageSampleTime and messageDeliveryInfos.size() < numMessageDeliveryInfos)
		messageDeliveryInfos.push_back(info);
}



void KadObserver::registerPacketDelivered(KadDataPacket& packet) {
	if (not packet.isTracked()) return;
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	deliveredDataPackets++;
	const OverlayKey fromKey = packet.getSrcNodeID();
	const OverlayKey toKey = packet.getDstNodeID();
	if (not exists(fromKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	MessageDeliveryInfo info{};
	info.hops = getR2Kademlia(fromKey).getHopLimitDefault()-packet.getHopLimit();
	info.distance = getPhysicalDistance(fromKey, toKey);
	info.prefixDist = fromKey.getLength() - fromKey.sharedPrefixLength(toKey);
	info.overlayHops = packet.getOverlayHops();
	
	ASSERT(info.overlayHops > 0);
	histograms[hopCountData]->collect(info.hops);
	histograms[distanceData]->collect(info.distance);
	histograms[stretchMulData]->collect(info.hops / (double)info.distance);
	histograms[stretchAddData]->collect(info.hops - info.distance);
	histograms[prefixDistData]->collect(info.prefixDist);
	histograms[overlayHopsData]->collect(info.overlayHops);
	histograms[prefixPerHopData]->collect(info.prefixDist / (double)info.overlayHops);
	if (simTime() > messageSampleTime and dataDeliveryInfos.size() < numMessageDeliveryInfos)
		dataDeliveryInfos.push_back(info);
}


void KadObserver::registerPacketOverlayHop(KadPacket& packet) {
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	packet.countOverlayHop();
}


// calculate stretch from answer packet
void KadObserver::sampleAnswer(KadAnswer& packet) {
	// answer packet has reverse path
	const OverlayKey toKey = packet.getSource();
	const OverlayKey fromKey = packet.getTarget();
	if (not exists(fromKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	uint distance= getPhysicalDistance(fromKey, toKey);
	uint hops= packet.getProgress();

	//EV_TRACE << "sampleAnswer: hops=" << hops << " distance=" << distance << endl;
	if (distance > 0)
	{
		histograms[stretchMulAns]->collect(hops / (double)distance);
		histograms[stretchAddAns]->collect(hops - distance);
	}
}


// calculate stretch from answer packet if shortened as much as possible
// assumption is that path does not contain the own node ID and is not the reverse path
void KadObserver::sampleLaterPath(const OverlayKey& originKey, const kad_path& path) {
	const OverlayKey toKey = path.back();
	if (not exists(originKey) or not exists(toKey)) {
		EV_ERROR << "missing keys on packet" << endl;
		return;
	}
	uint distance= getPhysicalDistance(originKey, toKey);
	uint hops= path.size();

	EV_TRACE << "sampleLaterPath: hops=" << hops << " distance=" << distance << endl;
	if (distance > 0)
	{
		histograms[stretchMulLater]->collect(hops / (double)distance);
		histograms[stretchAddLater]->collect(hops - distance);
	}
}



void KadObserver::sampleStatistics (Statistics &stats) {
	// records every histogram mean as corresponding vector
	for (uint i=0; i<histograms.size(); i++)
		if (histograms[i]->getCount() > 0)
			outVectors[i]->record(histograms[i]->getMean());
	// delivery ratio control packets
	if (deliveredPackets > 0)
		addOutVectors[deliveryRatioV]->record(deliveredPackets/((double)deliveredPackets + droppedPackets));
	
	// delivery ratio data packets
	if (deliveredDataPackets > 0)
		addOutVectors[deliveryRatioDataV]->record(deliveredDataPackets/((double)deliveredDataPackets + droppedDataPackets));

	addOutVectors[deliveredDataPacketsV]->record(deliveredDataPackets);
	addOutVectors[droppedDataPacketsV]->record(droppedDataPackets);
	addOutVectors[roamingDataPacketsV]->record(registeredDataPackets - deliveredDataPackets - droppedDataPackets);

	// reset, but not if simulation hits the run time limit
	if (resetAfterSample and simTime()<simTimeLimit) {
		for (auto& h : histograms)
			h->clear();
		deliveredPackets = 0;
		droppedPackets = 0;
		registeredPackets = 0;
		deliveredDataPackets = 0;
		droppedDataPackets = 0;
		registeredDataPackets = 0;
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
