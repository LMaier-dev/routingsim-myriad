@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");

cplusplus {{
#include "BaseTypes.h"
#include "NodePkt_m.h"
}}

namespace routingsim;

enum MyriadPDUType {
    MYRIAD_HELLO = 0x01;
    MYRIAD_DD = 0x02;
    MYRIAD_LSR = 0x03;
    MYRIAD_LSU = 0x04;
    MYRIAD_ACK = 0x05;
}

packet MyriadPacket {
    MyriadPDUType type;
    nodeid_t source;
}

packet MyriadHello extends MyriadPacket {
    type = MYRIAD_HELLO;
    nodeid_t knownNeighbors[];
    std::string rings;
}