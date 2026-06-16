@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");

import foundation.NodePkt;

cplusplus {{
#include "BaseTypes.h"
#include "NodePkt_m.h"

namespace routingsim {
// new "types"

} // end namespace

}}

// the following will be defined in namespace routingsim
namespace routingsim;

enum MyriadGpPDUType {
    MYRIADGP_HELLO = 0;
    MYRIADGP_COLLECT = 1;
    MYRIADGP_UPDATE = 2;
    MYRIADGP_DEGRADE = 3;
}

// directly from Kira (mirrors MyriadLsPkt.msg)
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

packet MyriadGpHeader {
    MyriadGpPDUType type;
    // router_id_src/ringking: I receive my packet back -> ack in current ring
    nodeid_t router_id_src;
    // Also I only have one inflight packet for each ring so we have simple data structure of retransmission timers for each ring we are in.
    // for external forwards we may need another data structure to cache packets or retx
    uint32_t seq_no;
    uint64_t random_no; // for dedup
    uint64_t current_ring; // only fwd unconditionally here
}

packet MyriadGpCollect extends MyriadGpHeader {
    type = MYRIADGP_COLLECT;
    uint64_t nlri[]; // subnets each node inserts its own subnetsOUTSIDE
}

packet MyriadGpUpdate extends MyriadGpCollect {
    type = MYRIADGP_UPDATE;
    nodeid_t chain[]; // one chain per update as the whole ring share subnet reachability
    // when we forward externally we attach current_ring to the chain
    // and also set router_id_src (ringking) to our id then send reliable
}

packet MyriadGpDegrade extends MyriadGpHeader {
    type = MYRIADGP_DEGRADE;
    // TODO: I think theres no path needed since I dont think we need loop prevention here (since we dedup I think we may not)
    uint64_t ring;
}
