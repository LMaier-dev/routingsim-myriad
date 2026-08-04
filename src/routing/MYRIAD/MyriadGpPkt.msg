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
    // Ring member count: king seeds 1, each forwarder increments; on the king's
    // own-return it equals the number of routers on the ring (its size).
    uint32_t member_count;
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
    // Rings already flooded into on this lap, so a second bridge node of a
    // double transition doesn't duplicate the flood. Accumulates around the
    // ring; reset (empty) on inter-ring transition.
    uint32_t handled_rings[];
    // Ring member count (not exponentiated) NED knob. On the origin UPDATE, ring_chain is empty and member_count below
    // carries the origin ring's size instead.
    uint32_t chain_sizes[];
    // Origin ring's member count (origin UPDATE trailer only; like widths).
    uint32_t member_count;
}

// One inter-ring UPDATE transition carried inside a batch. Every entry in a
// batch shares the same owner-king and target ring, so those live on the batch
// header (router_id_src, current_ring); the entry carries only the per-chain
// fields (seq_no, ring_chain, spof_count, rr, chain_sizes) plus its OWN
// handled_rings — each entry transitions into different other rings as the
// batch laps, so the "already flooded this lap" marks must be per-entry, not
// shared. widths/member_count ride only origin UPDATEs. (A class, not a struct:
// OMNeT++ structs can't hold dynamic-array fields.)
class MyriadGpUpdateEntry {
    uint32_t seq_no;
    uint32_t ring_chain[];
    uint32_t spof_count;
    MyriadGpRr rr[];
    uint32_t chain_sizes[];
    // Rings this entry has already been flooded into on the current lap (per-
    // entry analogue of MyriadGpUpdate.handled_rings). Reset empty at birth.
    uint32_t handled_rings[];
}

// Batch of inter-ring UPDATE transitions, all owned by router_id_src and all
// transitioning into current_ring, coalesced within one holdInterval window
// (see the holdInterval NED knob). The batch laps current_ring WHOLE — never
// unpacked intra-ring — and closes a single circular-ACK (matched on
// router_id_src + current_ring + seq_no) for all its entries. seq_no is the
// owner's per-target-ring batch-lap counter, distinct from per-entry seq_no.
packet MyriadGpUpdateBatch extends MyriadGpHeader {
    type = MYRIADGP_UPDATE_BATCH;
    MyriadGpUpdateEntry entries[];
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
