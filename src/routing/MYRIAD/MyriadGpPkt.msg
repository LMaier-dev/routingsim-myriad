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
    uint32_t current_ring; // only fwd unconditionally here
}

// tracks if the neighbor ring of the currentRing is connected over two routers or just one
struct MyriadGpRingTransition {
    uint32_t ring;
    bool isRedundant;
}

// (subnet, owner) pair: owner is the router that inserted the subnet during
// collect. Enables segment-routing lookup: subnet -> owner -> chain.
struct MyriadGpRr {
    uint32_t subnet;
    nodeid_t owner;
}

packet MyriadGpNLRI extends MyriadGpHeader {
    type = MYRIADGP_COLLECT;
    MyriadGpRr rr[]; // each node inserts its own (subnet, myId) pairs
}

packet MyriadGpCollect extends MyriadGpNLRI {
    // every collection the ringking checks if transition widths changed
    type = MYRIADGP_COLLECT;
    MyriadGpRingTransition transitions[];
}

packet MyriadGpUpdate extends MyriadGpNLRI {
    type = MYRIADGP_UPDATE;
    // Prepend-order chain of ring ids: chain[0] = latest ring visited,
    // chain.back() = origin ring. Empty at origination (origin = current_ring).
    uint32_t ring_chain[];
    // Single-point-of-failure count along the chain (single-node transitions).
    uint32_t spof_count;
    // Transition-width trailer: the origin king's adjacentRings for current_ring.
    // Non-empty only on the origin UPDATE; members store it and strip on
    // inter-ring forward.
    MyriadGpRingTransition widths[];
}

// DEGRADE is a direct-neighbor flood, not tied to the ring machinery.
// Own root packet — does NOT extend MyriadGpHeader (no current_ring).
packet MyriadGpDegrade {
    MyriadGpPDUType type = MYRIADGP_DEGRADE;
    nodeid_t router_id_src;   // last forwarder (for skip-sender flood)
    uint32_t ring_id;         // culprit (broken) ring
    uint32_t seq_no;          // culprit ring's seq at time of failure
    uint32_t beacon_no;       // bumped by originator every beacon interval
}
