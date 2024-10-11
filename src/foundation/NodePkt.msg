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

// NodePkt attributes (symbolic names for bits)
enum NodePktAttributes {
     NODEPKT_ATTR_NONE = 0;
     NODEPKT_ATTR_NO_PROCESSING_DELAY = 1;
     NODEPKT_ATTR_DO_NOT_COUNT_ON_LINK = 2;
     NODEPKT_ATTR_BIT_2 = 4;
     NODEPKT_ATTR_BIT_3 = 8;
     NODEPKT_ATTR_BIT_4 = 16;
     NODEPKT_ATTR_BIT_5 = 32;
     NODEPKT_ATTR_BIT_6 = 64;
     NODEPKT_ATTR_BIT_7 = 128;
     NODEPKT_ATTR_BIT_8 = 256;
}

packet NodePkt {
    int type @enum(PayloadTypes);
    int attributes; // bitfield of packet attributes, bits 0..15 reserved for foundation, upper bits for protocol use
    hop_t hop;
    int destGateId = -1;
}
