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

#include "R2Kademlia.h"

namespace routingsim {

Define_Module(R2Kademlia);

void R2Kademlia::onInitialize() {
	EV << "R2Kademlia::onInitialize()" << endl;
}

void R2Kademlia::onConnect( nodeid_t nid ) {
	EV << "Now I am connected to " << nid << endl;
}

void R2Kademlia::onDisconnect( nodeid_t nid ) {
	EV << "Now I am disconnected from " << nid << endl;
}

void R2Kademlia::onMessage( NodePkt_p pkt ) {
	EV << "received msg" << endl;
	EV << "unhandled packet: " << pkt->getType() << endl;
}

void R2Kademlia::reset() {
	EV << "resetting state of R2Kademlia" << endl;
}

} //namespace
