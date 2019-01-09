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
#include "RPC.h"
#include <memory>
}}

namespace routingsim;

enum KadPacketType {
	KAD_INVALID = 0;
	KAD_HELLO = 1;
	KAD_PING = 2;
	KAD_PONG = 3;
}

class noncobject OverlayKey;
class noncobject nodeid_t;
class noncobject nonce_t;

packet KadPacket {
	@customize(true);
	OverlayKey route[];
	uint16_t progress = 0;
	nonce_t nonce;
	uint8_t type enum(KadPacketType);
}
