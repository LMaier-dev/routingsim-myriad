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
#include <memory>
}}

namespace routingsim;

enum KadPacketType {
    INVALID = 0;
	HELLO = 1;
    PING = 2;
    PONG = 3;
}

class noncobject OverlayKey;

packet KadPacket {
	OverlayKey target;
	OverlayKey source;
	unsigned int nonce;
	uint8_t type enum(KadPacketType);
}

packet KadAnswer extends KadPacket {
}

packet KadHello extends KadPacket {
	type = HELLO;
}

packet KadPing extends KadPacket {
	type = PING;
}

packet KadPong extends KadAnswer {
	type = PONG;
}

