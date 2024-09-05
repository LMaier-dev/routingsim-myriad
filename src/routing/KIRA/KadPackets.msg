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
import KadPacket;

cplusplus {{
#include "KadPacket.h"
#include "stateseqno.h"

namespace routingsim {
using key_vector_t = std::vector<routingsim::OverlayKey>;
}

}}

namespace routingsim;

class key_vector_t { @existingClass; }

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


// currently nothing special for response packets
// just to better distinguish them in the code
packet KadAnswer extends KadPacket {
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
	age_t pathInfoAge[];
	nodedegree_t nodeDegree[];
	bool PNListDelivered = false;
}


// FIND NODE
// tries to reach dstNodeID via Overlay routing strategy
packet KadFindNodeReq extends KadRTableReq {
	type = KAD_FindNodeReq;

	OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY;
	bool exact = false;
	bool rediscovery = false;
}


packet KadFindNodeRsp extends KadRTableRsp {
	type = KAD_FindNodeRsp;
}

/// UpdateRouteReq (w/o response)
packet KadUpdateRouteReq extends KadPacket {
       type = KAD_UpdateRouteReq;

       OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY;

       OverlayKey contact[];
       key_vector_t path[];
       stateseqno_t seqNo[];
       age_t pathInfoAge[];
       nodedegree_t nodeDegree[];
       uint8_t action[] @enum(KadRouteUpdateAction);
}


// ERROR
packet KadError extends KadAnswer {
	type = KAD_Error;
	uint8_t errorType @enum(KadErrorType);
	// that is the final destination of the error
	OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY;
	// this denotes the link that failed: <packet source, failedNextHopNodeID>
	OverlayKey failedNextHopNodeID = OverlayKey::UNSPECIFIED_KEY;
	// origDstNodeID is the original destination of the data packet in forward direction
	OverlayKey origDstNodeID = OverlayKey::UNSPECIFIED_KEY;
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
	OverlayKey phyNeighbor[];
	stateseqno_t phyNeighborSeqNo[];
	age_t phyNeighborAge[];
	nodedegree_t phyNeighborDegree[];
}

// PNDiscRsp
packet KadPNDiscRsp extends KadRTableRsp {
	type = KAD_PNDiscRsp;
}



// PROBE
packet KadProbeReq extends KadPacket {
	type = KAD_ProbeReq;
	bool pathCheck = false;
}

packet KadProbeRsp extends KadAnswer {
	type = KAD_ProbeRsp;
}


// PATHSETUP
packet KadPathSetupReq extends KadPacket {
	type = KAD_PathSetupReq;
	// transmit contact state at the originator
	simtime_t contactLastSeen;
	stateseqno_t contactSeqNo;
}

packet KadPathSetupRsp extends KadAnswer {
	type = KAD_PathSetupRsp;
	// this echoes the contact state at the originator
	// so that the originator can check whether this Rsp is obsolete
	simtime_t contactLastSeen;
	stateseqno_t contactSeqNo;
	// need to record the complete original path as Rsp is sent back earlier
	OverlayKey fwdSourcePath[];
}


// PATHTEARDOWN
// this is a kind of optimization as soft states will also remove orphans
packet KadPathTeardownReq extends KadPacket {
	type = KAD_PathTeardownReq;
}


// QUERY ROUTE
// request routing table information from node
// in contast to FindNodeReq only sent to contacts directly (no overlay routing)
packet KadQueryRouteReq extends KadRTableReq {
	type = KAD_QueryRouteReq;
}

packet KadQueryRouteRsp extends KadRTableRsp {
	type = KAD_QueryRouteRsp;
}


// Reverse Flood along PathIDs
packet KadRevPathFlood extends KadPacket {
        type = KAD_RevPathFlood;

	// List of PathIDs for further reverse path flooding of this message
	PathID PIDList[];
}


packet KadLinkFailureNotify extends KadPacket {
        type = KAD_LinkFailureNotify;

	// the final destination of the message
	OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY;
	// PathID to reach first segment sources if applicable
	PathID dstPID = PathID::UndefinedPathID;
}