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

#ifndef __ROUTINGSIM_NETWORKMANAGER_H_
#define __ROUTINGSIM_NETWORKMANAGER_H_

#include <omnetpp.h>
#include <unordered_set>

/// forward declarations
namespace routingsim {
class NetworkManager;
NetworkManager* getNetworkManager();
}

#include "NetworkTopology.h"
#include "NetworkDynamics.h"
#include "../mobility/MobilityTopology.h"
#include "../mobility/MobilityDynamics.h"
#include "StatisticSample.h"

namespace routingsim {

class NetworkManager: public cSimpleModule, public StatisticSample {
public:
	/// Returns the network topology  // TODO: mobilityTopology
	inline NetworkTopology& getTopology() {
          if (useMobility)
            return *mobilityTopology;
          else
            return *topology;
        }

	/// Returns the network dynamics module
	inline NetworkDynamics& getDynamics() { return *dynamics; }

        /// Returns the mobility manager
        inline MobilityTopology& getMobilityTopology() { return *mobilityTopology; }

        /// Returns the mobility dynamics module
        inline MobilityDynamics& getMobilityDynamics() { return *mobilityDynamics; }

	/// Returns a list of currently failed or affected nodes
	inline const std::unordered_set<nodeid_t>& getFailedNodes() const { return failedNodes; }

	inline bool useMobilityDynamics() const { return useMobility and mobilityDynamics; }

        /// Returns the network manager singleton
	static inline NetworkManager& getManager() { return *getNetworkManager(); }

	/// Destructor, reset singleton
	~NetworkManager();

protected:
	virtual void initialize();
	virtual void finish();
	virtual void handleMessage(cMessage *msg);
	void sampleStatistics( Statistics& stats );

private:
	NetworkTopology* topology = nullptr;
	NetworkDynamics* dynamics = nullptr;
    MobilityTopology* mobilityTopology = nullptr;
    MobilityDynamics* mobilityDynamics = nullptr;

	std::unordered_set<nodeid_t> failedNodes; // this is set by DynamicsInfo messages
    bool useMobility = false;

	// Topology Statistics
	enum uint8_t {
		topostatsmode_none,
		topostatsmode_light,
		topostatsmode_heavy
	} topologyStatisticsMode;

    bool evaluateStatistics = true;
	uint64_t edgeChangesCount = 0;
	uint64_t nodeChangesCount = 0;
    cOutVector *activeNodesCountV = nullptr;
	cOutVector *activeEdgesCountV = nullptr;
	cOutVector *partitionCount = nullptr;

	time_t startTime;
	time_t stopTime;
};

/// Returns the network Manager singleton
NetworkManager* getNetworkManager();

} //namespace

#endif
