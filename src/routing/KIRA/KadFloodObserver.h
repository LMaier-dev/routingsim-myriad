#ifndef __ROUTINGSIM_KadFloodObserver_H_
#define __ROUTINGSIM_KadFloodObserver_H_

#include <boost/bimap.hpp>
#include <cstdint>
#include <vector>
#include "BaseTypes.h"
#include "StatisticSample.h"
#include "cmodule.h"
#include "KadPackets_m.h"
#include "OverlayKey128.h"
#include "simtime_t.h"

namespace routingsim {
using cHistogramPtr = std::unique_ptr<cHistogram>;
using linkMap_t = std::map<uint64_t, uint32_t>;

struct FloodState {
    uint32_t inflightCount = 0;
    uint32_t deliveredCount = 0;
    uint32_t droppedCount = 0;
    uint32_t forwardedCount = 0;

    uint32_t packetsForwarded = 0;

    nodeid_t source;
    simtime_t originTime;

    // the hops to reach each node
    cHistogramPtr hops;
    // the overlayhops to reach each
    cHistogramPtr overlayHops;
    // the replicationCount 
    cHistogramPtr replications;
    // the stretch
    cHistogramPtr stretch;
    // the latency
    cHistogramPtr latency;

    // I probably need to hold state about each node:
    //   - path length
    //   - overlayhop
    //   - prefixLength
    //   - arrivalTime
    //
    // I probably need to hold state about each link aswell (for a link usage metric)
    //   - It will probably be easiest to reconstruct the replication tree and draw all statistics from that
    linkMap_t linkMapBD;
    linkMap_t linkMapUD;
};


/**
 * Functionality to collect global Statistics regarding individual Floods
 **/
class KadFloodObserver: public cSimpleModule, public StatisticSample {
protected:
	virtual void initialize() override;
	virtual void finish() override;
	virtual void handleMessage(cMessage *msg) override;
public:
    KadFloodObserver() {
           floodStates = std::vector<FloodState>(); 
    }
    KadFloodObserver(const KadFloodObserver&) = delete; 
private:
    std::vector<FloodState> floodStates;

public:
    // Create a new floodInstance
    uint32_t registerFlood(const OverlayKey& source, nodeid_t nid);
    void registerFloodForwarded(uint32_t floodId, const OverlayKey& source, nodeid_t nid, const KadFloodPacket& packet, hop_t takenHop);
    void registerFloodReplication(uint32_t floodId, const OverlayKey& overlay_hop, nodeid_t nid, uint8_t floodPrefixLength, uint16_t replicationCount);
    void registerFloodDropped(uint32_t floodId, const OverlayKey& hop, nodeid_t nid, const KadFloodPacket& packet);
    void registerFloodDelivered(uint32_t floodId, const OverlayKey& hop, nodeid_t nid, const KadFloodPacket& packet);

	virtual void sampleStatistics (Statistics &stats) override;
	void recordFloodStatistics();
};

KadFloodObserver& getKadFloodObserver();

cHistogramPtr getIntegerHistogram(const char* name);
cHistogramPtr getDoubleHistogram(const char* name);

} // namespace

#endif
