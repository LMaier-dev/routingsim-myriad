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
#include "stateseqno.h"
#include "PathID.h"
#include "RPC.h"
#include <memory>
#include <utility>
typedef std::pair<OverlayKey,OverlayKey> link_t;

typedef uint32_t age_t;
typedef uint16_t nodedegree_t;
}}

namespace routingsim;

enum KadPDUType {
	KAD_PNHello = 		0x01;
	KAD_PNDiscReq = 	0x03;
	KAD_PNDiscRsp = 	0x04;

	KAD_FindNodeReq = 	0x09;
	KAD_FindNodeRsp = 	0x0a;
	KAD_QueryRouteReq = 	0x0b;
	KAD_QueryRouteRsp = 	0x0c;
	KAD_UpdateRouteReq = 	0x0d;

	KAD_Error =		0x71;
	KAD_ProbeReq = 		0x81;
	KAD_ProbeRsp = 		0x82;

	KAD_PathSetupReq = 	0x83;
	KAD_PathSetupRsp = 	0x84;

	KAD_PathTeardownReq = 	0x85;
	KAD_PathTeardownRsp = 	0x86;

	KAD_Data = 		0x10;
}

class noncobject OverlayKey;
class noncobject nodeid_t;
class noncobject nonce_t;
class noncobject link_t;
class noncobject stateseqno_t;
class noncobject age_t;
class noncobject nodedegree_t;

// a KAD control packet
packet KadPacket {
	@customize(true);
	OverlayKey route[];
	bool active[]; // simulator internal only
	uint16_t progress = 0;
	uint16_t hopLimit = 255;
	// state seq number from source node
	stateseqno_t stateSeqNo= stateseqno_t::invalidSeqNo;
	nodedegree_t srcNodeDegree;
	nonce_t nonce= 0; // simulator internal only
	uint8_t type enum(KadPDUType);
	bool notVia = false;
	link_t failedLink[];
	age_t failedLinkAge[]; // age of link info in ms
}

class noncobject PathID;

// a fast path Data packet
packet KadDataPacket {
	@customize(true);
	PathID firstPathID = PathID::UndefinedPathID;
	PathID secondPathID = PathID::UndefinedPathID;
	OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY;
	OverlayKey srcNodeID = OverlayKey::UNSPECIFIED_KEY;
	uint8_t type enum(KadPDUType);
	uint16_t hopLimit = 255;
}
