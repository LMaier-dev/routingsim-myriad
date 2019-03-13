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

#ifndef __ROUTINGSIM_KadObserver_H_
#define __ROUTINGSIM_KadObserver_H_

#include <boost/bimap.hpp>
#include "foundation.h"
#include "OverlayKey.h"

namespace routingsim {

class MessageStatistics;

/**
 * The R2/Kademlia Global Observer
 */
class KadObserver: public cSimpleModule {
protected:
	virtual void initialize() override;
	virtual void finish() override;
	virtual void handleMessage(cMessage *msg) override;

private:
	boost::bimap<OverlayKey, nodeid_t> nodes;

public:

	// register / unregister normal ring identifiers
	void registerUp(const OverlayKey& key, nodeid_t owner);
	void registerDown(const OverlayKey& key, nodeid_t owner);

};

KadObserver& getKadObserver();

} // namespace

#endif
