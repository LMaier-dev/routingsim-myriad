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

#ifndef __ROUTINGSIM_TEMPLATEPROTOCOL_H_
#define __ROUTINGSIM_TEMPLATEPROTOCOL_H_

#include <omnetpp.h>

#include "foundation.h"
#include "StatisticSample.h"
#include "routing/KIRA/KIRA.h"

namespace routingsim {

/**
 * Implementation of the a template for a Protocol
 *
 * Goals:
 * (1) ...
 * (2) ...
 */
class KIRALSR : public Protocol
{
	
  static const int KIRALSR_PACKET_TYPE = 9;

  protected:
  	/// called at initialization time of the Protocol
  	void onInitialize();
  	/// called when a new node is connected
  	void onConnect( nodeid_t nid );
  	/// called when a node is disconnected
  	void onDisconnect( nodeid_t nid );
  	/// on message
  	void onMessage( NodePkt_p pkt );
    void onSelfMessage(cMessage_p msg);
	/// reset state of Protocol to initial state (Node Failure)
	void reset();

    void addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback);

    template<typename T>
    std::unique_ptr<T> makePacket(const char *name);

  private: 
    R2Kademlia* kira = nullptr;
};

inline void KIRALSR::addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback) {
	auto msg = new TimerMsg(-1, callback);
	// callback will be carried out after delay
	scheduleAt(simTime() + delay, msg);
}

template<typename T>
std::unique_ptr<T> KIRALSR::makePacket(const char *name) {
	auto packet = std::make_unique<T>();
	packet->setName(name);
	// TODO optimization: do this in one pass
	return packet;
}

} //namespace

#endif
