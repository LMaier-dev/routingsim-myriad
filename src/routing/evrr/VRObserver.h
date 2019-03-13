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

#ifndef __ROUTINGSIM_VROBSERVER_H_
#define __ROUTINGSIM_VROBSERVER_H_

namespace routingsim {
class VRObserver;
}

#include <omnetpp.h>
#include <vector>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include "foundation.h"
#include "VirtualRing.h"

namespace routingsim {

using namespace std;

class MessageStatistics;

/**
 * The Virtual Ring Global Observer
 */
class VRObserver: public cSimpleModule {
protected:
	virtual void initialize();
	virtual void finish();
	virtual void handleMessage(cMessage *msg);

private:
	// virtual ids
	vector<vid_t> entries;
	MessageStatistics* general;

	// anycast support
	typedef boost::unordered_multimap<vid_t,nodeid_t> anycastmap_t;
	vector<vid_t> anycast_entries;
	anycastmap_t anycast_map;
	MessageStatistics* anycast;

	// message statistics
	cMessage* timer;

public:
	// return virtual ring neighbors
	void getNeighbors(boost::unordered_set<vid_t>& set, vid_t id, size_t k);

	// register / unregister normal ring identifiers
	void up(vid_t id, nodeid_t owner = undefined_node );
	void down(vid_t id, nodeid_t owner = undefined_node );

	// return a random identifier
	vid_t random() const;

	// return a random anycast identifier
	vid_t random_anycast() const;

	// delivery ratio and stretch
	void messageRegister( cMessage* msg, bool anycast = false );
	void messageDelivered( cMessage* msg, vid_t dest, distance_t dist );
	void messageDrop( cMessage* msg );
};

VRObserver& getVRObserver();

} // namespace

#endif
