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

#include "PathVector.h"
#include "PathVectorPkt_m.h"

#include <string>
#include <sstream>

#define CSTR(X,Y) \
	std::ostringstream __##X; __##X << Y; \
	const char* X = __##X.str().c_str()

#define BUBBLE(X,Y) CSTR(X,Y); node().bubble(X);

namespace routingsim {

Define_Module(PathVector);

void PathVector::onInitialize() {
	EV << "Path Vector!" << endl;
}

/// called when a new node is connected
void PathVector::onConnect( nodeid_t nid ) {
	BUBBLE(s, "Connected to " << nid );
}

/// called when a node is disconnected
void PathVector::onDisconnect( nodeid_t nid ) {
	BUBBLE(s, "Disconnected to " << nid );
}

/// on message
void PathVector::onMessage( NodePkt* pkt ) {
	BUBBLE(s, "Message from " << pkt->getHop().from );
}

void PathVector::reset() {
	BUBBLE(s, "resetting node" );
}

} //namespace
