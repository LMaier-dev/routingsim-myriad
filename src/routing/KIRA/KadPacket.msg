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
@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");

import foundation.NodePkt;

cplusplus {{
#include "BaseTypes.h"
#include "OverlayKey.h"
#include "stateseqno.h"
#include "PathID.h"
#include "RPC.h"
#include <memory>
#include <utility>
#include "KadTypes.h"

namespace routingsim {
// new "types"

using link_t = std::pair<routingsim::OverlayKey,routingsim::OverlayKey>;

} // end namespace

}}

// the following will be defined in namespace routingsim
namespace routingsim;

// in case you add new PDU types here, please also add them in KadPacket.cc PDUNames
enum KadPDUType {
	KAD_ULNHello   = 0;
	KAD_ULNDiscReq = 1;
	KAD_ULNDiscRsp = 2;

	KAD_FindNodeReq = 3;
	KAD_FindNodeRsp = 4;
	KAD_QueryRouteReq = 5;
	KAD_QueryRouteRsp = 6;
	KAD_UpdateRouteReq = 7;

	KAD_Error = 8;
	KAD_RevPathFlood = 9;
	KAD_LinkFailureNotify = 10;
	KAD_ProbeReq = 11;
	KAD_ProbeRsp = 12;

	KAD_PathSetupReq = 13;
	KAD_PathSetupRsp = 14;

	KAD_PathTeardownReq = 15;

	KAD_Data = 16;
	KAD_Flood = 17;
	// count is only a helper, but required
	count = 18;
};

class OverlayKey extends cObject {
 @existingClass;
}


class nodeid_t {
      @cppType(routingsim::nodeid_t);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue((omnetpp::intval_t)($));
      @fromValue(omnetpp::checked_int_cast<unsigned short>($.intValue()));
}

class nonce_t {
      @cppType(routingsim::nonce_t);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue((omnetpp::intval_t)($));
      @fromValue(omnetpp::checked_int_cast<unsigned short>($.intValue()));
}

class link_t { @existingClass; } // that's a container
class stateseqno_t { @existingClass; } // that's an actual class

class nodedegree_t {
      @cppType(routingsim::nodedegree_t);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue((omnetpp::intval_t)($));
      @fromValue(omnetpp::checked_int_cast<unsigned short>($.intValue()));
}

class age_t {
      @cppType(routingsim::age_t);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue((omnetpp::intval_t)($));
      @fromValue(omnetpp::checked_int_cast<unsigned short>($.intValue()));
}

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
	uint8_t type @enum(KadPDUType);
	bool notVia = false;
	link_t failedLink[];
	age_t failedLinkAge[]; // age of link info in ms
}


// PathID required for data packets
class PathID {
 @existingClass;
}

class NodePkt_ps { @existingClass; } // that's a container

// a fast path Data packet
packet KadDataPacket {
	@customize(true);
	PathID firstPathID = PathID::UndefinedPathID;             // outer header destination (outermost destination)
	PathID secondPathID = PathID::UndefinedPathID;            // outer header destination
	OverlayKey pathSrcNodeID = OverlayKey::UNSPECIFIED_KEY;   // outer header source (previous overlay hop)
	OverlayKey dstNodeID = OverlayKey::UNSPECIFIED_KEY; // inner header
	OverlayKey srcNodeID = OverlayKey::UNSPECIFIED_KEY; // inner header
	uint8_t type enum(KadPDUType);
	uint16_t hopLimit = 255;
    NodePkt_ps payload = nullptr;
}
