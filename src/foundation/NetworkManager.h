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
#include <boost/unordered_map.hpp>

/// forward declarations
namespace routingsim {
class NetworkManager;
NetworkManager* getNetworkManager();
}

#include "NetworkTopology.h"
#include "NetworkDynamics.h"

namespace routingsim {

class NetworkManager: public cSimpleModule {
public:
	/// Returns the distance between two modules in the network topoology
	distance_t getDistance(cModule *from, cModule* to);

	/// Returns current size of the network
	/// TODO: atm returns the number of nodes disregarding their status!
	int getNetworkSize();

	/// Returns the network topology
	inline NetworkTopology& getTopology() { return *topology; }

	/// Returns the network dynamics module
	inline NetworkDynamics& getDynamics() { return *dynamics; }

	/// Returns the network manager singleton
	static inline NetworkManager& getManager() { return *getNetworkManager(); }

	/// Destructor, reset singleton
	~NetworkManager();

	NetworkManager();

protected:
	virtual void initialize();
	virtual void finish();
	virtual void handleMessage(cMessage *msg);

private:
	NetworkTopology* topology;
	NetworkDynamics* dynamics;
	void resetNode(nodeid_t node);
};

/// Returns the network Manager singleton
NetworkManager* getNetworkManager();

} //namespace

#endif
