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
typedef uint16_t stateseqno_t;
typedef uint32_t age_t;
}}

namespace routingsim;
class KadPacket extends cPacket;
class KadDataPacket extends cPacket;
class noncobject nodeid_t;
class noncobject OverlayKey;
class noncobject key_vector_t;
class noncobject stateseqno_t;
class noncobject age_t;

enum KadErrorType {
	KAD_NoError = 0;
	KAD_NodeUnreachable = 0x01;
	// Malformed message 0x02 // not necessary in simulation
	// Parameter problem 0x03 // not necessary in simulation
	KAD_HopLimitExceeded = 0x04;
	KAD_SegmentFailure = 0x05;
	KAD_PathIDUnknown = 0x06;
	KAD_RouteFailureDeadEnd = 0x0a;
}

enum KadRouteUpdateAction {
     KAD_Announce = 0;
     KAD_Withdraw = 1;
     KAD_Change   = 2;
     KAD_Unreachable = 3;
}

packet KadAnswer extends KadPacket {
	bool shortened = false; // true if return path was shortened (removed cycles)
}

/// compare RTableRequestType from Spec
packet KadRTableReq extends KadPacket {
	// reserved flag is omitted
	bool contactsOnly = false; // return contacts only
	bool overlayNeighbors = false; // return k nearest overlay neighbors of the node
	bool overlayNeighborsSource = false; // return k nearest overlay neighbors known for the source node
	uint8_t neighborhood = 63; // physical l-hop neighborhood, 63 means full table
}

/// compare RTable or ContactList from Spec
/// if ContactList, all other array stay empty
packet KadRTableRsp extends KadAnswer {
	// count is clear
	OverlayKey contact[];
	key_vector_t path[];
	stateseqno_t seqNo[];
	age_t contactAge[];
	// bool unknownFlag[];
	// bool intermediateFlag[];
}

/// UpdateRouteReq (w/o response)
packet KadUpdateRouteReq extends KadPacket {
       type = KAD_UpdateRouteReq;

       OverlayKey dstNodeID;

       OverlayKey contact[];
       key_vector_t path[];
       stateseqno_t seqNo[];
       age_t contactAge[];
       uint8_t action[] enum(KadRouteUpdateAction);
}

// KadDataMsg
packet KadDataMsg extends KadDataPacket {
       type = KAD_Data;
       bool redirected;
}
// ------------

// ERROR
packet KadError extends KadAnswer {
	type = KAD_Error;
	uint8_t errorType enum(KadErrorType);
}

// HELLO
// compare PNHello
packet KadHello extends KadPacket {
	type = KAD_PNHello;
	nodeid_t sourceId;
}


// PNDiscReq
packet KadPNDiscReq extends KadRTableReq {
	type = KAD_PNDiscReq;
	nodeid_t sourceId;
}

// PNDiscRsp
packet KadPNDiscRsp extends KadRTableRsp {
	type = KAD_PNDiscRsp;
}



// PROBE
packet KadProbeReq extends KadPacket {
	type = KAD_ProbeReq;
}

packet KadProbeRsp extends KadAnswer {
	type = KAD_ProbeRsp;
}


// PATHSETUP
packet KadPathSetupReq extends KadPacket {
	type = KAD_PathSetupReq;
}

packet KadPathSetupRsp extends KadAnswer {
	type = KAD_PathSetupRsp;
	OverlayKey fwdSourcePath[];
}


// PATHTEARDOWN
packet KadPathTeardownReq extends KadPacket {
	type = KAD_PathTeardownReq;
}

packet KadPathTeardownRsp extends KadAnswer {
	type = KAD_PathTeardownRsp;
}


// QUERY ROUTE
packet KadQueryRouteReq extends KadRTableReq {
	type = KAD_QueryRouteReq;
}

packet KadQueryRouteRsp extends KadRTableRsp {
	type = KAD_QueryRouteRsp;
}


// FIND NODE
packet KadFindNodeReq extends KadRTableReq {
	type = KAD_FindNodeReq;

	OverlayKey dstNodeID;
	bool exact = false;
	bool rediscovery = false;
}


packet KadFindNodeRsp extends KadRTableRsp {
	type = KAD_FindNodeRsp;
}
