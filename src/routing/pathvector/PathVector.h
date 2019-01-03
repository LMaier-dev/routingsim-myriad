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

#ifndef __ROUTINGSIM_PATHVECTOR_H_
#define __ROUTINGSIM_PATHVECTOR_H_

#include <omnetpp.h>
#include <vector>

#include "foundation.h"

namespace routingsim {

/**
 * TODO - Generated class
 */
class PathVector: public Protocol {
protected:
	/// initialize
	virtual void onInitialize();

	/// called when a new node is connected
	virtual void onConnect( nodeid_t nid );

	/// called when a node is disconnected
	virtual void onDisconnect( nodeid_t nid );

	/// on message
	virtual void onMessage( NodePkt* pkt );
	/// reset state of Protocol to initial state (Node Failure)
	void reset();

private:
	seqnum_t syn;
	class route_t {
	public:
		path_t route;
		nodeid_t owner;
		vector_set<nodeid_t> used_by;
	};
	std::vector<route_t> routes;
};

} //namespace

#endif
