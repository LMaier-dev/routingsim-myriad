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
#include "routing/evrr/VRObserver.h"
#include "StatisticSample.h"

namespace routingsim {

/**
 * Implementation of the a template for a Protocol
 *
 * Goals:
 * (1) ...
 * (2) ...
 */
class TemplateProtocol : public Protocol
{
  protected:
  	/// called at initialization time of the Protocol
  	void onInitialize();
  	/// called when a new node is connected
  	void onConnect( nodeid_t nid );
  	/// called when a node is disconnected
  	void onDisconnect( nodeid_t nid );
  	/// on message
  	void onMessage( NodePkt* pkt );
	/// reset state of Protocol to initial state (Node Failure)
	void reset();
};

} //namespace

#endif
