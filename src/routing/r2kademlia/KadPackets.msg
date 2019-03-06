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

cplusplus {{
#include "BaseTypes.h"
#include "OverlayKey.h"
#include "KadPacket.h"
#include <memory>
#include <vector>
class KadPacket;
typedef std::vector<OverlayKey> key_vector_t;
}}

namespace routingsim;
class KadPacket extends cPacket;
class noncobject nodeid_t;
class noncobject OverlayKey;
class noncobject key_vector_t;

enum KadErrorType {
	KAD_PACKET_INVALID = 0;
	KAD_NEXT_HOP_UNAVAILABLE = 1;
}

packet KadAnswer extends KadPacket {}

/// compare RTableRequestType from Spec
packet KadRTableReq extends KadPacket {
	// reserved flag is ommited
	bool contactsOnly = false;
	uint8_t neighborhood = 63;
}

/// compare RTable or ContactList from Spec
/// if ContactList, all other array stay empty
packet KadRTableRsp extends KadAnswer {
	// count is clear
	OverlayKey contact[];
	key_vector_t path[];
	bool unknownFlag[];
	bool intermediateFlag[];
}
//TODO: KadUpdateRouteReq
//TODO: KadDataMsg

// ------------

// ERROR
packet KadError extends KadAnswer {
	type = KAD_ERROR;
	uint8_t errorType enum(KadErrorType);
}

// HELLO
// compare PHello
packet KadHello extends KadPacket {
	type = KAD_HELLO;
	nodeid_t sourceId;
}

// PROBE
packet KadProbeReq extends KadPacket {
	type = KAD_PING;
}
packet KadProbeRsp extends KadAnswer {
	type = KAD_PONG;
}

// GET NEIGHBORS
packet KadQueryRouteReq extends KadRTableReq {
	type = KAD_GET_NEIGHBORS_CALL;
}

packet KadQueryRouteRsp extends KadRTableRsp {
	type = KAD_GET_NEIGHBORS_ANSWER;
}


// FIND NODE
packet KadFindNodeReq extends KadRTableReq {
	type = KAD_FIND_NODE_CALL;
	//TODO use destId for this
	OverlayKey query;
}
packet KadFindNodeRsp extends KadRTableRsp {
	type = KAD_FIND_NODE_ANSWER;
}
