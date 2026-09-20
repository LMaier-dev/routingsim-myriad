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

enum MyriadPvPDUType {
    MYRIADPV_HELLO = 0;
    MYRIADPV_COLLECT = 1;
    MYRIADPV_UPDATE = 2;
    MYRIADPV_DEGRADE = 3;
    MYRIADPV_UPDATE_BATCH = 4;
}

// What a COLLECT lap is for. REFRESH and ELECTION both crown a king and run the
// contention rules; only REFRESH re-announces the ring. The rest exist so churn
// never originates an UPDATE.
enum MyriadPvCollectKind {
    // Takes a vacant king's seat. Elects but does not announce: the new king's
    // own refresh does that, keeping origination refresh-paced.
    COLLECT_ELECTION = 0;
    // Liveness check, and the lap a router with no ring state sends to join.
    // Forwarded unconditionally; its budget expiry fires DEGRADE.
    COLLECT_PROBE = 1;
    // The only kind that makes a member replay its RIB into the ring.
    COLLECT_REPLAY = 2;
    // The king's periodic refresh, and the only kind that announces the ring.
    COLLECT_REFRESH = 3;
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

packet MyriadPvHeader {
    MyriadPvPDUType type;
    // A packet lapping back to its router_id_src on current_ring is the ACK.
    nodeid_t router_id_src;
    uint32_t seq_no;
    uint32_t current_ring; // forwarded unconditionally within this ring
    uint8_t hops = 0;
}

// isRedundant: false = a single-router (SPOF) transition to `ring`, true = two+.
struct MyriadPvRingTransition {
    uint32_t ring;
    bool isRedundant;
}

// (subnet, owner) pair, owner being the router that inserted the subnet during
// collect. Enables segment-routing lookup: subnet -> owner -> chain.
struct MyriadPvRr {
    uint32_t subnet;
    nodeid_t owner;
}

packet MyriadPvNLRI extends MyriadPvHeader {
    type = MYRIADPV_COLLECT;
    MyriadPvRr rr[]; // each node inserts its own (subnet, myId) pairs
}

packet MyriadPvCollect extends MyriadPvNLRI {
    type = MYRIADPV_COLLECT;
    MyriadPvRingTransition transitions[];
    // King seeds 1, each forwarder increments; on own-return it equals the
    // ring's size.
    uint32_t member_count;
    // Originator lacked state (memberCount==0): loses the king election to any
    // live member (formation tie-break by id).
    bool originator_crashed;
    // Best replay source seen on this lap: origin count first, then fewer total
    // spofs, then lower total cost over the preferred chains. A forwarder must
    // beat all three to take the slot, so it never trades breadth for quality.
    // Not part of the election.
    uint32_t lap_max_routes;
    uint32_t lap_max_spof;
    uint64_t lap_max_cost;   // SUM over a replay source's origins: stays 64-bit
    // Who holds the figures above.
    nodeid_t lap_max_owner;
    // The FEWEST destination rings any member on this lap holds.
    uint32_t lap_min_routes;
    // Set by the one member that replayed on this REPLAY lap; makes the rest
    // defer, and tells the originator on return that a replay was sent.
    bool replay_claimed;
    // See MyriadPvCollectKind.
    uint8_t kind = COLLECT_ELECTION;
}

packet MyriadPvUpdate extends MyriadPvNLRI {
    type = MYRIADPV_UPDATE;
    // Prepend-order ring ids (see RibEntry in MyriadPv.h); empty at origination.
    uint32_t ring_chain[];
    uint32_t spof_count;
    // Origin king's adjacentRings for current_ring; set only on the origin
    // UPDATE, members store it and strip it on inter-ring forward.
    MyriadPvRingTransition widths[];
    // Rings already flooded into on this lap, so the second bridge of a double
    // transition doesn't duplicate; reset empty on inter-ring transition.
    uint32_t handled_rings[];
    uint32_t chain_cost;      // sum of memberCount^e over ring_chain; accumulated per-hop
    // Origin ring's size (origin UPDATE trailer only, like widths); on the
    // origin UPDATE ring_chain is empty so this carries the size instead.
    uint32_t member_count;
}

// One transition inside a batch. Owner and target ring live on the batch header;
// each entry carries the per-chain fields plus its OWN handled_rings, since
// entries fan into different rings as the batch laps. (A class not a struct:
// OMNeT++ structs can't hold dynamic arrays.)
class MyriadPvUpdateEntry {
    uint32_t seq_no;
    uint32_t ring_chain[];
    uint32_t spof_count;
    MyriadPvRr rr[];
    uint32_t chain_cost;
    uint32_t handled_rings[];
}

// Batch of inter-ring transitions sharing one owner and target ring, coalesced
// within a holdInterval window (see MyriadPv.ned). Laps current_ring WHOLE and
// closes a single ACK for all entries; seq_no is the owner's per-ring batch-lap
// counter (batchLapSeq in MyriadPv.h), distinct from the per-entry seq_no.
packet MyriadPvUpdateBatch extends MyriadPvHeader {
    type = MYRIADPV_UPDATE_BATCH;
    MyriadPvUpdateEntry entries[];
    // Ring trailer, carried by REPLAY batches only (replayInto) and once for
    // the whole batch: Same pair as the origin-UPDATE trailer; 0/empty on an ordinary batch.
    uint32_t member_count;
    MyriadPvRingTransition widths[];
}

// Direct-neighbor flood, not tied to the ring machinery, so no current_ring.
packet MyriadPvDegrade {
    MyriadPvPDUType type = MYRIADPV_DEGRADE;
    nodeid_t router_id_src;   // last forwarder (for skip-sender flood)
    uint32_t ring_id;         // culprit (broken) ring
    uint32_t seq_no;          // culprit ring's seq at time of failure
    uint32_t beacon_no;       // bumped by originator every beacon interval
}
