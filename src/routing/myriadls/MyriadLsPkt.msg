@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");

import foundation.NodePkt;

cplusplus {{
#include "BaseTypes.h"
#include "NodePkt_m.h"
#include "MyriadLsTypes.h"

namespace routingsim {
// new "types"

} // end namespace

}}

// the following will be defined in namespace routingsim
namespace routingsim;

enum MyriadPDUType {
    MYRIAD_HELLO = 0;
    MYRIAD_DBD = 1;
    MYRIAD_LSR = 2;
    MYRIAD_LSU = 3;
    MYRIAD_ACK = 4;
}

// directly from Kira
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

class r_chain {
      @cppType(routingsim::r_chain);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue($.c_str());
      @fromValue($.stringValue());
}

packet MyriadHeader {
    MyriadPDUType type;
    nodeid_t router_id_src;
}

packet MyriadHello extends MyriadHeader {
    type = MYRIAD_HELLO;
    simtime_t helloInterval;
    simtime_t deadInterval;
    nodeid_t knownNeighbors[];
    r_chain rings;
}

packet MyriadDBD extends MyriadHeader {
    type = MYRIAD_DBD;
}