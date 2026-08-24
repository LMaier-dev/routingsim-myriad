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
    MYRIADGP_UPDATE_BATCH = 4;
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
    // A packet lapping back to its router_id_src on current_ring is the ACK.
    nodeid_t router_id_src;
    uint32_t seq_no;
    uint32_t current_ring; // forwarded unconditionally within this ring
}

// isRedundant: false = a single-router (SPOF) transition to `ring`, true = two+.
struct MyriadGpRingTransition {
    uint32_t ring;
    bool isRedundant;
}

// (subnet, owner) pair, owner being the router that inserted the subnet during
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
    type = MYRIADGP_COLLECT;
    MyriadGpRingTransition transitions[];
    // King seeds 1, each forwarder increments; on own-return it equals the
    // ring's size.
    uint32_t member_count;
    // Originator lacked state (memberCount==0): loses election to any live node.
    bool originator_crashed;
    // OR'd by any forwarder lacking state; tells the king to replay on own-return.
    bool crashed_present;
}

packet MyriadGpUpdate extends MyriadGpNLRI {
    type = MYRIADGP_UPDATE;
    // Prepend-order ring ids (see RibEntry in MyriadGp.h); empty at origination.
    uint32_t ring_chain[];
    uint32_t spof_count;
    // Origin king's adjacentRings for current_ring; set only on the origin
    // UPDATE, members store it and strip it on inter-ring forward.
    MyriadGpRingTransition widths[];
    // Rings already flooded into on this lap, so the second bridge of a double
    // transition doesn't duplicate; reset empty on inter-ring transition.
    uint32_t handled_rings[];
    uint64_t chain_cost;      // sum of memberCount^e over ring_chain; accumulated per-hop
    // Origin ring's size (origin UPDATE trailer only, like widths); on the
    // origin UPDATE ring_chain is empty so this carries the size instead.
    uint32_t member_count;
}

// One transition inside a batch. Owner and target ring live on the batch header;
// each entry carries the per-chain fields plus its OWN handled_rings, since
// entries fan into different rings as the batch laps. (A class not a struct:
// OMNeT++ structs can't hold dynamic arrays.)
class MyriadGpUpdateEntry {
    uint32_t seq_no;
    uint32_t ring_chain[];
    uint32_t spof_count;
    MyriadGpRr rr[];
    uint64_t chain_cost;
    uint32_t handled_rings[];
}

// Batch of inter-ring transitions sharing one owner and target ring, coalesced
// within a holdInterval window (see MyriadGp.ned). Laps current_ring WHOLE and
// closes a single ACK for all entries; seq_no is the owner's per-ring batch-lap
// counter (batchLapSeq in MyriadGp.h), distinct from the per-entry seq_no.
packet MyriadGpUpdateBatch extends MyriadGpHeader {
    type = MYRIADGP_UPDATE_BATCH;
    MyriadGpUpdateEntry entries[];
}

// Direct-neighbor flood, not tied to the ring machinery, so no current_ring.
packet MyriadGpDegrade {
    MyriadGpPDUType type = MYRIADGP_DEGRADE;
    nodeid_t router_id_src;   // last forwarder (for skip-sender flood)
    uint32_t ring_id;         // culprit (broken) ring
    uint32_t seq_no;          // culprit ring's seq at time of failure
    uint32_t beacon_no;       // bumped by originator every beacon interval
}
