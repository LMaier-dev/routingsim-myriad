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

#ifndef __ROUTINGSIM_NETWORKDYNAMICS_H_
#define __ROUTINGSIM_NETWORKDYNAMICS_H_

#include <omnetpp.h>

// forward declarations
namespace routingsim {
class NetworkDynamics;
}

#include "NetworkTopology.h"
#include "NetworkManager.h"

namespace routingsim {

/**
 * This class generates dynamics in the network topology.
 *
 * @author Christoph Werle <werle@kit.edu>
 */
class NetworkDynamics: public cSimpleModule {
	friend class NetworkManager;
private:
	cMessage* timer;
	NetworkTopology* topology;
	NetworkManager* manager;
	virtual void handleMessage(cMessage *msg);
    virtual void initialize(int stage);
    virtual void finish();
    virtual int numInitStages() const;

protected:
    /**
     * Initialize module.
     */
    virtual void initialize();

	/**
	 * Sub-classes need to implement this function to schedule dynamic events.
	 */
	virtual simtime_t scheduleDynamics() = 0;

	/**
	 * Schedules a node failure at the given time and duration.
	 * This is a convenience method.
	 */
	void scheduleNodeFailure(nodeid_t node, simtime_t occurence, simtime_t duration );

public:
	inline NetworkTopology& getTopology() { return *topology; }
	inline NetworkManager& getManager() { return *manager; }
};

/**
 * Dynamic information handled by the network manager
 */
class DynamicsInfo {
public:
	// dynamic types
	enum type_t {
		nodeFailure, linkFailure, nodeRecovery, linkRecovery, undefined = 0xff
	};
	type_t type;

	// affected nodes
	nodeid_t source, destination;

	// occurence and duration
	simtime_t occurrence, duration;

	DynamicsInfo() :
		type(undefined), source(undefined_node), destination(undefined_node),
				occurrence(0), duration(0) {
	}
};

} //namespace

#endif
