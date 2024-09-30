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
}}

namespace routingsim;

class hop_t {
      @existingClass;
}


// additional types can be defined by the protocol module (see Protocol::bind())
enum PayloadTypes {
    PAYLOAD_INVALID = 0;
    PAYLOAD_HELLO = 1;
    PAYLOAD_GOODBYE = 2;
}

packet NodePkt {
    int type @enum(PayloadTypes);
    int attribute; // bitfield of packet attributes, bits 0..15 reserved for foundation, upper bits for protocol use
    hop_t hop;
    int destGateId = -1;
}
