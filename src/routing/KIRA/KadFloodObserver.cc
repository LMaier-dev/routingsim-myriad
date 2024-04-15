#include "KadFloodObserver.h"
#include "BaseTypes.h"
#include "NetworkManager.h"
#include "chistogram.h"
#include "cmodule.h"
#include "csimulation.h"
#include "KadPackets_m.h"
#include <boost/optional/optional.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

namespace routingsim {
Define_Module(KadFloodObserver);

/// singleton pointer
static KadFloodObserver* floodObserver = NULL;

void KadFloodObserver::initialize() {
	ASSERT(floodObserver == NULL || floodObserver == this);
	getStatistics().addSample(this);
}

void KadFloodObserver::finish() {
	recordFloodStatistics();
    floodStates.clear();

	ASSERT(floodObserver == this);
	floodObserver = NULL;
}


void KadFloodObserver::handleMessage(cMessage *msg) {
	delete msg;
}

uint32_t KadFloodObserver::registerFlood(const OverlayKey& source, nodeid_t nid, std::optional<hop_t> event_id) {
    auto floodId = floodStates.size();
    FloodState state = {};
    state.hops = cHistogramPtr(getIntegerHistogram(("hops" + std::to_string(floodId)).c_str()));
    state.overlayHops = cHistogramPtr(getIntegerHistogram(("overlayHops" + std::to_string(floodId)).c_str()));
    state.replications = cHistogramPtr(getIntegerHistogram(("replications" + std::to_string(floodId)).c_str()));
    state.stretch = cHistogramPtr(getDoubleHistogram(("stretch" + std::to_string(floodId)).c_str()));
    state.latency = cHistogramPtr(getDoubleHistogram(("latency" + std::to_string(floodId)).c_str()));
    
    // init delivery map for event_id if not none
    if (event_id) {
        auto hop = event_id.value();
        auto linkId = (((uint64_t) hop.from) << (uint64_t) 32) | (uint64_t) hop.to; 
        state.linkMapBD[linkId]++;
        if (hop.from > hop.to) {
            linkId = (((uint64_t) hop.to) << (uint64_t)  32) | (uint64_t) hop.from;
        }

        state.event_id.emplace(linkId);
        eventDeliveryMap[linkId].insert(nid);
    }

    state.source = nid;
    state.originTime = simTime();

    floodStates.push_back(std::move(state));
    return floodId;
}

void KadFloodObserver::registerFloodReplication(uint32_t floodId, const OverlayKey& overlay_hop, nodeid_t nid, uint8_t floodPrefixLength, uint16_t replicationCount) {
    auto& state = floodStates.at(floodId);
    state.replications->collect(replicationCount);
    state.inflightCount += replicationCount;
}

void KadFloodObserver::registerFloodDelivered(uint32_t floodId, const OverlayKey& hop, nodeid_t nid, const KadFloodPacket& packet) {
    auto& state = floodStates.at(floodId);

    state.inflightCount--;
    state.deliveredCount++;
    // todo make this configurable 
    auto hopCount = 256 - packet.getHopLimit();
    state.hops->collect(hopCount);
    state.overlayHops->collect(packet.getOverlayHops());
    state.stretch->collect(hopCount / (double) getNetworkManager()->getTopology().getDistance(state.source, nid));
    state.latency->collect(simTime() - state.originTime);
    if (state.event_id) {
        eventDeliveryMap[*state.event_id].insert(nid);
    }
}

void KadFloodObserver::registerFloodDropped(uint32_t floodId, const OverlayKey& hop, nodeid_t nid, const KadFloodPacket& packet) {
    auto& state = floodStates.at(floodId);

    state.inflightCount--;
    state.droppedCount++;
}

void KadFloodObserver::registerFloodForwarded(uint32_t floodId, const OverlayKey& hop, nodeid_t nid, const KadFloodPacket& packet, hop_t takenHop) {
    auto& state = floodStates.at(floodId);

    // build a unqiue linkid 
    auto linkId = (((uint64_t) takenHop.from) << (uint64_t) 32) | (uint64_t) takenHop.to; 
    state.linkMapBD[linkId]++;
    if (takenHop.from > takenHop.to) {
        linkId = (((uint64_t) takenHop.to) << (uint64_t)  32) | (uint64_t) takenHop.from;
    }
    state.linkMapUD[linkId]++;
    state.forwardedCount++;
}

void KadFloodObserver::recordFloodStatistics() {
	cModuleType* modType = cModuleType::get("routingsim.routing.r2kademlia.Dummy");
    uint32_t i = 0;
    for (auto& flood: floodStates) {
		cModule* mod = modType->create(("Flood" + std::to_string(i)).c_str(), this);
        mod->recordStatistic(flood.replications.get());
        mod->recordStatistic(flood.hops.get());
        mod->recordStatistic(flood.overlayHops.get());
        mod->recordStatistic(flood.stretch.get());
        mod->recordStatistic(flood.latency.get());
        // linkStatistics
        auto linkUsageBD = getIntegerHistogram(("linkUsageBD" + std::to_string(i)).c_str());
        auto linkUsageUD = getIntegerHistogram(("linkUsageUD" + std::to_string(i)).c_str());
        for (auto const& k: flood.linkMapBD) {
            linkUsageBD->collect(k.second);
        }
        mod->recordStatistic(linkUsageBD.get());
        for (auto const& k: flood.linkMapUD) {
            linkUsageUD->collect(k.second);
        }
        mod->recordStatistic(linkUsageUD.get());
        i++;
    }
    i = 0;
    for (auto& event: eventDeliveryMap) {
		cModule* mod = modType->create(("Event" + std::to_string(i)).c_str(), this);
        mod->recordScalar("deliveryCount", std::get<1>(event).size());
        i++;
    }
}

void KadFloodObserver::sampleStatistics (Statistics &stats) {
}

cHistogramPtr getIntegerHistogram(const char *name) {
	auto hist = std::make_unique<cHistogram>(name);
	hist->setRange(0, NAN);
	hist->setNumBinsHint(1000);
	hist->setMode(cHistogram::MODE_INTEGERS);
	return hist;
}

cHistogramPtr getDoubleHistogram(const char *name) {
	auto hist = std::make_unique<cHistogram>(name);
	hist->setRange(0.0, NAN);
	hist->setNumBinsHint(1000);
	hist->setMode(cHistogram::MODE_DOUBLES);
	return hist;
}

KadFloodObserver& getKadFloodObserver() {
	if (floodObserver == NULL) {
		cModuleType* type = cModuleType::get(
				"routingsim.routing.r2kademlia.KadFloodObserver");
		cModule* mod = getSimulation()->getModuleByPath("<root>");
		floodObserver = dynamic_cast<KadFloodObserver*>(type->create("KadFloodObserver", mod));
	}
	return *floodObserver;
}

} // namespace
