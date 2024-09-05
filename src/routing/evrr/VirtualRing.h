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

#ifndef __ROUTINGSIM_VIRTUALRING_H_
#define __ROUTINGSIM_VIRTUALRING_H_

#include <omnetpp.h>
#include <vector>


#include "foundation.h"

namespace routingsim {

/// virtual identifier definitions
using vid_t = uint32_t;
const vid_t undefined_vid = 0;

}

#include "VRObserver.h"
#include "StatisticSample.h"

namespace routingsim {

/**
 * The implementation of a Virtual Ring Routing (VRR)-inspired protocol.
 *
 * The primary differences to the original VRR by Matthew Caesar et al. is that
 *
 * (1) It uses a pro-active distance vector protocol to disseminate
 *     new routes sparsely. Furthermore, sequence numbers are used to
 *     eliminate loops in routing (e.g., like DSDV, or Babel).
 *
 * (2) The distance-vector scheme implements a case of linearization, thus,
 *     double-rings are inhibited.
 *
 * @author Sebastian Mies <mies@gtfr.de>
 */
class VirtualRing: public LegacyProtocol, public StatisticSample {
protected:
 	/// initialize
	virtual void onInitialize() override;

	/// called when a new node is connected
	virtual void onConnect( nodeid_t nid ) override;

	/// called when a node is disconnected
	virtual void onDisconnect( nodeid_t nid ) override;

	/// on message
	virtual void onMessage( NodePkt* pkt ) override;

	/// on self message (timer)
	virtual void onSelfMessage( cMessage* msg ) override;

	/// on timer message
	void onTimer();

	/// reset state of Protocol to initial state (Node Failure)
	void reset() override;

	/// sample statistics
	virtual void sampleStatistics( Statistics& stats ) override;

private:
	// routing table
	class Table;
	Table* table;

	/// round timer
	cMessage* timer;

	// parameters
	simtime_t roundTime;
	unsigned int vneighborTimeout;
	bool immediateResponse;
	bool useRouteDescentCriterion;
	unsigned int consistencyCheck;
	unsigned int timeout;

	simtime_t sendMessageDelay;

	// state
	bool connected;
	int consistencyCheckCount;

	// observer and stats
	VRObserver* observer;
	uintmax_t controlMessagesIn;
	uintmax_t controlMessagesInLast;
	uintmax_t controlMessagesOut;
	uintmax_t controlMessagesOutLast;
	uintmax_t controlTrafficIn;
	uintmax_t controlTrafficInLast;
	uintmax_t controlTrafficOut;
	uintmax_t controlTrafficOutLast;
	uintmax_t routeUpdatesSent;
	uintmax_t routeUpdatesReceived;
	uintmax_t tableVNeighborChanges;
	uintmax_t tableChanges;
	uintmax_t seqChanges;

private:
	void sendProbeMessage(bool anycast);
	void flushBacklog();
	void route( NodePkt* );
	void visualize( bool );
	vid_t get_random_id();
	vid_t get_anycast_id(vid_t id);
};

} // namespace

#endif
